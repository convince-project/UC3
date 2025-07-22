/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "ExecuteDanceComponent.h"
#include <fstream>
#include <nlohmann/json.hpp>
#include <cmath>
#include <map>
#include <sstream>

/**
 * @brief Initializes the ExecuteDanceComponent
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return true if initialization succeeded, false otherwise
 * 
 * This method:
 * 1. Initializes ROS2 if not already active
 * 2. Connects to DanceComponent to get robot part names
 * 3. Creates YARP ports to communicate with ctpService (joint-level control)
 * 4. Creates ROS2 services for external interface
 * 5. Initializes cartesian controller for pointing towards artworks
 * 6. Loads POI coordinates for robot orientation
 */
bool ExecuteDanceComponent::start(int argc, char*argv[])
{
    // Initialize ROS2 if not already initialized
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    
    // === SETUP CONNECTIONS WITH CTP SERVICE ===
    // The ctpService (Control Trajectory Planner Service) manages joint movements
    // Each robot part (head, arms, torso) has its dedicated ctpService
    
    // Create temporary node to call GetPartNames service
    auto getPartNamesClientNode = rclcpp::Node::make_shared("ExecuteDanceComponentGetPartNamesNode");
    std::shared_ptr<rclcpp::Client<dance_interfaces::srv::GetPartNames>> getPartNamesClient =
        getPartNamesClientNode->create_client<dance_interfaces::srv::GetPartNames>("/DanceComponent/GetPartNames");
    
    // Prepare request to get all controllable robot parts
    auto getPartNamesRequest = std::make_shared<dance_interfaces::srv::GetPartNames::Request>();
    
    // Wait for service to be available (with timeout)
    while (!getPartNamesClient->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'getPartNamesClient'. Exiting.");
        }
    }
    
    // Send request and wait for response synchronously
    auto getPartNamesResult = getPartNamesClient->async_send_request(getPartNamesRequest);
    rclcpp::spin_until_future_complete(getPartNamesClientNode, getPartNamesResult);
    auto ctpServiceParts = getPartNamesResult.get()->parts;
    
    // For each robot part (e.g., head, left_arm, right_arm, torso)
    // create a YARP port to communicate with the relative ctpService
    if (!ctpServiceParts.empty())
    {
        for (std::string part : ctpServiceParts)
        {
            // Dynamically allocate a new YARP port for this part
            yarp::os::Port *ctpPort = new yarp::os::Port;
            std::string portName = "/ExecuteDanceComponentCtpServiceClient/" + part + "/rpc";
            
            // Open port with unique name for each part
            bool b = ctpPort->open(portName);
            if (!b)
            {
                yError() << "Cannot open" << part << " ctpService port";
                return false;
            }
            
            // Save port in map for future use (key=part_name, value=port)
            m_pCtpService.insert({part, *ctpPort});
            
            // Automatically connect to the specific ctpService of that part
            // The ctpService receives movement commands (joint positions, times, velocities)
            yarp::os::Network::connect(portName, "/ctpservice/" + part + "/rpc");
        }
    }
    else
    {
        yWarning() << "Movement part names are empty. No movements will be executed!";
    }
    
    // === SETUP ROS2 SERVICES ===
    // Create the main ROS2 node for this component
    m_node = rclcpp::Node::make_shared("ExecuteDanceComponentNode");
    
    // Create service to execute a dance/movement
    // Other components (e.g., DialogueComponent) call this service
    m_executeDanceService = m_node->create_service<execute_dance_interfaces::srv::ExecuteDance>("/ExecuteDanceComponent/ExecuteDance",  
                                                                                std::bind(&ExecuteDanceComponent::ExecuteDance,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    
    // Create service to check if robot is executing movements
    m_isDancingService = m_node->create_service<execute_dance_interfaces::srv::IsDancing>("/ExecuteDanceComponent/IsDancing",
                                                                                std::bind(&ExecuteDanceComponent::IsDancing,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));

    // === SETUP LOCALIZATION AND NAVIGATION ===
    // 1) Subscribe to AMCL topic to know current robot position
    // AMCL (Adaptive Monte Carlo Localization) provides robot pose in the map
    m_amclSub = m_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&ExecuteDanceComponent::amclPoseCallback, this, std::placeholders::_1));

    // 2) Load POI (Points of Interest) coordinates from JSON file
    // POIs represent artworks and their positions in the map
    m_poiCoords = loadPoiCoordinates("conf/board_coords.json");

    // === SETUP CARTESIAN CONTROLLER ===
    // 3) Initialize client for YARP cartesian controller
    // The cartesian controller manages movements in 3D space (x,y,z + orientation)
    // Used to make robot arm/hand point towards artworks
    {
        yarp::os::Property opts;
        opts.put("device","cartesiancontrollerclient");                    // YARP device type
        opts.put("local", "/ExecuteDanceComponent/cartesian:o");           // Local port name
        opts.put("remote","/cartesianController/rpc:i");                   // Remote controller port name
        
        // Open device and get control interface
        m_cartesianClient.open(opts);
        if (m_cartesianClient.isValid()) {
            // Get ICartesianControl interface to send positioning commands
            m_cartesianClient.view(m_cartesianCtrl);
        }
    }

    RCLCPP_DEBUG(m_node->get_logger(), "ExecuteDanceComponent::start");      
    return true;
}

/**
 * @brief Closes the component and frees all resources
 * @return true if closing succeeded
 * 
 * Deallocates YARP ports and terminates ROS2
 */
bool ExecuteDanceComponent::close()
{
    // Close and deallocate all ctpService ports
    for (auto port : m_pCtpService)
    {
        delete &port.second;
    }
    
    // Close cartesian controller client
    if (m_cartesianClient.isValid()) {
        m_cartesianClient.close();
    }
    
    // Terminate ROS2
    rclcpp::shutdown();  
    return true;
}

/**
 * @brief Starts the main ROS2 loop
 * 
 * This method blocks and handles ROS2 service calls
 */
void ExecuteDanceComponent::spin()
{
    rclcpp::spin(m_node);  
}

/**
 * @brief Executes a complete dance/movement (main task)
 * @param request Request containing dance name or command
 * 
 * This is the component's core. It handles:
 * 1. Pointing towards artworks (if requested)
 * 2. Sequential execution of dance movements
 * 3. Coordination with DanceComponent for synchronization
 */
void ExecuteDanceComponent::executeTask(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request)
{
    // === ARTWORK POINTING MANAGEMENT ===
    // If dance name corresponds to a POI, orient robot towards artwork
    auto it = m_poiCoords.find(request->dance_name);
    if (it != m_poiCoords.end() && m_cartesianCtrl) {
        // Calculate direction towards artwork
        double dx     = it->second.first  - m_currentX;      // X difference
        double dy     = it->second.second - m_currentY;      // Y difference
        double goal   = std::atan2(dy, dx);                  // Target angle
        double dtheta = goal - m_currentYaw;                 // Angular difference
        
        // Normalize angle to interval [-π, π]
        while (dtheta > M_PI)  dtheta -= 2*M_PI;
        while (dtheta < -M_PI) dtheta += 2*M_PI;
        
        RCLCPP_INFO(m_node->get_logger(),
                    "Orientation towards POI '%s': dθ=%.2f rad",
                    request->dance_name.c_str(), dtheta);
        
        // Get current end-effector pose
        yarp::sig::Vector currentPos(3), currentOri(4);
        if (m_cartesianCtrl->getPose(currentPos, currentOri)) {
            // Modify only orientation to point towards artwork
            yarp::sig::Vector newOri = currentOri;
            newOri[2] = sin(dtheta/2);  // Z component of quaternion
            newOri[3] = cos(dtheta/2);  // W component of quaternion
            
            // Execute movement to new orientation
            m_cartesianCtrl->goToPoseSync(currentPos, newOri);
            m_cartesianCtrl->waitMotionDone();  // Wait for completion
        }
    }
    else {
        RCLCPP_WARN(m_node->get_logger(),
                    "No POI found for '%s', skipping orientation",
                    request->dance_name.c_str());
    }

    // === MAIN MOVEMENT EXECUTION LOOP ===
    // Get and execute movements one by one from DanceComponent
    bool done_with_getting_dance = false;
    do {
        // === GET NEXT MOVEMENT ===
        auto getMovementClientNode = rclcpp::Node::make_shared("ExecuteDanceComponentGetMovementNode");
        std::shared_ptr<rclcpp::Client<dance_interfaces::srv::GetMovement>> getMovementClient =
        getMovementClientNode->create_client<dance_interfaces::srv::GetMovement>("/DanceComponent/GetMovement");
        auto getMovementRequest = std::make_shared<dance_interfaces::srv::GetMovement::Request>();
        
        // Wait for service to be available
        while(!getMovementClient->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'getMovementClient'. Exiting.");
                return;
            }
        }
        
        // Get next movement in sequence
        auto getMovementResult = getMovementClient->async_send_request(getMovementRequest);
        rclcpp::spin_until_future_complete(getMovementClientNode, getMovementResult);
        auto movement = getMovementResult.get();
        
        // === EXECUTE MOVEMENT ===
        if (movement->is_ok == false) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "ExecuteDanceComponent::ExecuteDance. Movement not found, skipping...");
        } else {
            bool status;
            
            // Check type of movement to execute
            
            // Pointing movement towards artwork (new functionality)
            if (movement->part_name == "pointing_command") {
                status = ExecutePointingMovement(movement);
            }
            // Special movements (idle, navigation) -> immediate execution
            else if(m_danceName == "idleMove" || m_danceName == "navigationPosition") {
                // SendMovementNow interrupts ongoing movements and executes immediately
                status = SendMovementNow(movement->time, movement->offset, movement->joints, m_pCtpService.at(movement->part_name));
            } 
            // Normal dance movements -> added to queue
            else {
                // SendMovementToQueue adds movement to execution queue
                status = SendMovementToQueue(movement->time, movement->offset, movement->joints, m_pCtpService.at(movement->part_name));
            }
            
            if (!status) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Movement failed to sent. Is ctpService for part" << movement->part_name << "running?");
                continue;
            }
        }

        // === UPDATE MOVEMENT STATE ===
        // Inform DanceComponent that we have processed this movement
        auto updateMovementClientNode = rclcpp::Node::make_shared("ExecuteDanceComponentUpdateMovementNode");
        std::shared_ptr<rclcpp::Client<dance_interfaces::srv::UpdateMovement>> updateMovementClient =
        updateMovementClientNode->create_client<dance_interfaces::srv::UpdateMovement>("/DanceComponent/UpdateMovement");
        auto updateMovementRequest = std::make_shared<dance_interfaces::srv::UpdateMovement::Request>();
        
        while(!updateMovementClient->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'updateMovementClient'. Exiting.");
                return;
            }
        }
        
        // DanceComponent updates its internal state and tells us if dance is finished
        auto updateMovementResult = updateMovementClient->async_send_request(updateMovementRequest);
        rclcpp::spin_until_future_complete(updateMovementClientNode, updateMovementResult);
        auto updateMovementResponse = updateMovementResult.get();
        done_with_getting_dance = updateMovementResponse->done_with_dance;
        
    } while (!done_with_getting_dance);
}

/**
 * @brief Timer task to track dance duration
 * @param time Duration in seconds
 * 
 * This method runs in a separate thread and tracks how much time
 * has passed since dance start. Used to know if robot is still dancing.
 */
void ExecuteDanceComponent::timerTask(float time)
{
    // Set thread-safe flag indicating timer is active (robot is dancing)
    m_timerMutex.lock();
    m_timerTask = true;
    m_timerMutex.unlock();
    
    RCLCPP_INFO_STREAM(m_node->get_logger(), "Start Timer seconds: " << static_cast<int>(time));
    
    // Wait for specified duration
    std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(time)));

    // Timer finished, robot stopped dancing
    m_timerMutex.lock();
    m_timerTask = false;
    m_timerMutex.unlock();
    
    RCLCPP_INFO_STREAM(m_node->get_logger(), "End Timer ");
}

/**
 * @brief ROS2 service to execute a dance/movement
 * @param request Contains dance name to execute
 * @param response Confirms if operation succeeded
 * 
 * This is the main entry point. Other components call this service
 * to start a dance/movement on the robot.
 */
void ExecuteDanceComponent::ExecuteDance(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request,
             std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Response> response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::ExecuteDance " << request->dance_name);
    
    // === SET DANCE IN DANCE COMPONENT ===
    // First, tell DanceComponent which dance to load
    auto setDanceClientNode = rclcpp::Node::make_shared("ExecuteDanceComponentSetDanceNode");
    std::shared_ptr<rclcpp::Client<dance_interfaces::srv::SetDance>> setDanceClient =
    setDanceClientNode->create_client<dance_interfaces::srv::SetDance>("/DanceComponent/SetDance");
    auto setDanceRequest = std::make_shared<dance_interfaces::srv::SetDance::Request>();
    setDanceRequest->dance = request->dance_name;
    m_danceName = request->dance_name; // Save name locally for reference
    
    while (!setDanceClient->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setDanceClient'. Exiting.");
        }
    }
    
    auto setDanceResult = setDanceClient->async_send_request(setDanceRequest);
    rclcpp::spin_until_future_complete(setDanceClientNode, setDanceResult);
    auto setDanceResponse = setDanceResult.get();
    
    // If dance doesn't exist in DanceComponent, return error
    if (setDanceResponse->is_ok != true) {
        RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::ExecuteDance name: " << request->dance_name);
        response->is_ok = false;
        response->error_msg = "Dance not found";
        return;
    }
    
    // === START EXECUTION IN SEPARATE THREAD ===
    // If there's already a dance in progress, wait for it to finish
    if (m_threadExecute.joinable()) {
        m_threadExecute.join();
    }
    
    // Start new dance in separate thread
    // This allows service to return immediately while dance continues in background
    m_threadExecute = std::thread([this, request]() { executeTask(request); });
    
    response->is_ok = true;
}

/**
 * @brief ROS2 service to check if robot is dancing
 * @param request Empty request
 * @param response Contains true if robot is dancing
 * 
 * Other components can call this service to know if robot
 * is still busy with a dance.
 */
void ExecuteDanceComponent::IsDancing(const std::shared_ptr<execute_dance_interfaces::srv::IsDancing::Request> request,
             std::shared_ptr<execute_dance_interfaces::srv::IsDancing::Response> response) 
{
    (void)request;  // Suppress warning for unused parameter
    
    // Check timer state in thread-safe way
    m_timerMutex.lock();
    response->is_dancing = m_timerTask;
    m_timerMutex.unlock();
    
    RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::IsDancing " << response->is_dancing);
    response->is_ok = true;
}

/**
 * @brief Send movement to ctpService queue
 * @param time Movement duration in seconds
 * @param offset Time offset in milliseconds
 * @param joints Target joint positions (in degrees or radians)
 * @param port YARP port to specific ctpService
 * @return true if command was sent successfully
 * 
 * Queued movements are executed in sequence by ctpService.
 * This is the normal way to execute a smooth dance.
 */
bool ExecuteDanceComponent::SendMovementToQueue(float time, int offset, std::vector<float> joints, yarp::os::Port &port)
{
    yarp::os::Bottle res;
    yarp::os::Bottle cmd;
    
    // Build YARP command to add movement to queue
    cmd.addVocab32("ctpq");    // "Control Trajectory Planner Queue" - add to queue
    cmd.addVocab32("time");
    cmd.addFloat64(time);      // Movement duration
    cmd.addVocab32("off");
    cmd.addFloat64(offset);    // Time offset (delay before execution)
    cmd.addVocab32("pos");
    
    // List of target positions for each joint
    yarp::os::Bottle &list = cmd.addList();
    for (auto joint : joints)
    {
        list.addFloat64(joint);
    }
    
    // Send command to ctpService and wait for response
    return port.write(cmd, res);
}

/**
 * @brief Send movement for immediate execution
 * @param time Movement duration in seconds
 * @param offset Time offset in milliseconds  
 * @param joints Target joint positions
 * @param port YARP port to specific ctpService
 * @return true if command was sent successfully
 * 
 * Movement is executed immediately, interrupting any ongoing movements.
 * Used for special movements like idle or navigation positions.
 */
bool ExecuteDanceComponent::SendMovementNow(float time, int offset, std::vector<float> joints, yarp::os::Port &port)
{
    yarp::os::Bottle res;
    yarp::os::Bottle cmd;
    
    // Build YARP command for immediate execution
    cmd.addVocab32("ctpn");    // "Control Trajectory Planner Now" - execute immediately
    cmd.addVocab32("time");
    cmd.addFloat64(time);
    cmd.addVocab32("off");
    cmd.addFloat64(offset);
    cmd.addVocab32("pos");
    
    yarp::os::Bottle &list = cmd.addList();
    for (auto joint : joints)
    {
        list.addFloat64(joint);
    }
    return port.write(cmd, res);
}

/**
 * @brief Callback to update current robot position
 * @param msg AMCL message with robot pose
 * 
 * AMCL (Adaptive Monte Carlo Localization) publishes estimated
 * robot position in the map. This data is used to calculate orientation
 * towards artworks.
 */
void ExecuteDanceComponent::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // Extract cartesian position
    m_currentX   = msg->pose.pose.position.x;
    m_currentY   = msg->pose.pose.position.y;
    
    // Extract orientation (yaw) from quaternion
    double qx = msg->pose.pose.orientation.x,
           qy = msg->pose.pose.orientation.y,
           qz = msg->pose.pose.orientation.z,
           qw = msg->pose.pose.orientation.w;
    
    // Quaternion to Euler angle conversion (yaw only)
    m_currentYaw = std::atan2(2*(qw*qz + qx*qy),
                              1 - 2*(qy*qy + qz*qz));
}

/**
 * @brief Load POI coordinates from file
 * @param filename Name of file containing coordinates
 * @return Map with poi_name -> (x, y) coordinates
 * 
 * File must contain lines in format: poi_name x y
 * Example: quadro_1 2.5 1.0
 */
std::map<std::string, std::pair<double, double>> ExecuteDanceComponent::loadPoiCoordinates(const std::string& filename)
{
    std::map<std::string, std::pair<double, double>> poiMap;
    
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(m_node->get_logger(), "Cannot open POI file: %s", filename.c_str());
            return poiMap;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string name;
            double x, y;
            
            // Parsing: name x y
            if (iss >> name >> x >> y) {
                poiMap[name] = std::make_pair(x, y);
            }
        }
        
        RCLCPP_INFO(m_node->get_logger(), "Loaded %zu POIs from '%s'", poiMap.size(), filename.c_str());
        
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(m_node->get_logger(), "Error reading POI file '%s': %s", filename.c_str(), ex.what());
    }
    
    return poiMap;
}

/**
 * @brief Execute pointing movement towards specific artwork
 * @param movement Movement containing target artwork information
 * @return true if command was sent successfully
 * 
 * This is the new functionality for pointing. Handles cartesian
 * control of the arm to point towards specific artworks.
 */
bool ExecuteDanceComponent::ExecutePointingMovement(const std::shared_ptr<dance_interfaces::srv::GetMovement::Response> movement)
{
    // Extract artwork name from movement data
    std::string artworkName = extractArtworkName(movement->joints);
    
    RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::ExecutePointingMovement to: " << artworkName);
    
    // Hard-coded database of artwork coordinates (for initial testing)
    // In future these coordinates will come from YARP Map2D server
    std::map<std::string, std::vector<double>> artworkCoordinates = {
        {"quadro_1", {2.5, 1.0, 1.5}},     // x, y, z in map coordinates (meters)
        {"scultura_1", {4.0, 2.0, 1.2}},
        {"dipinto_1", {1.5, 3.5, 1.8}},
        {"statua_1", {3.0, 0.5, 1.0}}
    };
    
    // Search artwork in database
    auto it = artworkCoordinates.find(artworkName);
    if (it == artworkCoordinates.end()) {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Artwork not found: " << artworkName);
        return false;
    }
    
    // Get artwork coordinates and transform to robot reference frame
    std::vector<double> mapCoords = it->second;
    std::vector<double> robotCoords = transformMapToRobot(mapCoords);
    
    // Send command to cartesian controller
    return sendCartesianCommand(robotCoords);
}

/**
 * @brief Extract artwork name from movement data
 * @param joints Float vector encoding artwork name
 * @return Artwork name
 * 
 * Temporary implementation: uses first element of joints vector as index.
 * In future this could be a dedicated field in message or encoded differently.
 */
std::string ExecuteDanceComponent::extractArtworkName(const std::vector<float>& joints)
{
    // If no data, use default
    if (joints.empty()) return "quadro_1";
    
    // Use first joint as index in artwork array
    int index = static_cast<int>(joints[0]);
    std::vector<std::string> artworks = {"quadro_1", "scultura_1", "dipinto_1", "statua_1"};
    
    if (index >= 0 && index < artworks.size()) {
        return artworks[index];
    }
    
    return "quadro_1"; // fallback
}

/**
 * @brief Transform coordinates from map to robot reference frame
 * @param mapCoords x,y,z coordinates in map system
 * @return x,y,z coordinates relative to robot
 * 
 * This transformation is necessary because:
 * - Artworks are stored in map coordinates (fixed world system)
 * - Cartesian controller works in robot-relative coordinates
 * - Robot moves, so transformation changes dynamically
 */
std::vector<double> ExecuteDanceComponent::transformMapToRobot(const std::vector<double>& mapCoords)
{
    // TODO: In future use tf2 to get precise transformation
    // For now use current robot position from AMCL
    
    // Calculate difference vector between artwork and robot
    double dx = mapCoords[0] - m_currentX;      // X difference
    double dy = mapCoords[1] - m_currentY;      // Y difference
    double dz = mapCoords[2];                   // Absolute Z (artwork height)
    
    // Apply inverse rotation to get robot-relative coordinates
    // (rotate vector by negative robot orientation angle)
    double cos_theta = std::cos(-m_currentYaw);
    double sin_theta = std::sin(-m_currentYaw);
    
    double x_robot = dx * cos_theta - dy * sin_theta;
    double y_robot = dx * sin_theta + dy * cos_theta;
    double z_robot = dz - 1.0; // Subtract approximate robot base height
    
    RCLCPP_INFO_STREAM(m_node->get_logger(), 
        "Transformed coords - Map: [" << mapCoords[0] << ", " << mapCoords[1] << ", " << mapCoords[2] << 
        "] -> Robot: [" << x_robot << ", " << y_robot << ", " << z_robot << "]");
    
    return {x_robot, y_robot, z_robot};
}

/**
 * @brief Send positioning command to cartesian controller
 * @param coords Target x,y,z coordinates relative to robot
 * @return true if command was sent and accepted
 * 
 * Cartesian controller manages end-effector (hand/arm) movement
 * in 3D space, automatically calculating required joint angles
 * through inverse kinematics.
 */
bool ExecuteDanceComponent::sendCartesianCommand(const std::vector<double>& coords)
{
    // Verify cartesian controller is available
    if (!m_cartesianCtrl) {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Cartesian controller not available");
        return false;
    }
    
    // Prepare vectors for target position and orientation
    yarp::sig::Vector targetPos(3);
    targetPos[0] = coords[0];  // x (forward/backward relative to robot)
    targetPos[1] = coords[1];  // y (left/right relative to robot)
    targetPos[2] = coords[2];  // z (up/down relative to robot)
    
    // For now use natural arm orientation
    // TODO: Calculate orientation to point directly towards artwork
    yarp::sig::Vector targetOri(4);
    targetOri[0] = 0.0;  // x component of quaternion
    targetOri[1] = 0.0;  // y component of quaternion
    targetOri[2] = 0.0;  // z component of quaternion
    targetOri[3] = 1.0;  // w component of quaternion (neutral orientation)
    
    RCLCPP_INFO_STREAM(m_node->get_logger(), 
        "Sending cartesian command: [" << coords[0] << ", " << coords[1] << ", " << coords[2] << "]");
    
    // Send command to cartesian controller
    bool success = m_cartesianCtrl->goToPoseSync(targetPos, targetOri);
    
    if (success) {
        // Wait for movement completion
        m_cartesianCtrl->waitMotionDone();
        RCLCPP_INFO_STREAM(m_node->get_logger(), "Pointing command executed successfully");
        return true;
    } else {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Failed to send pointing command");
        return false;
    }
}
