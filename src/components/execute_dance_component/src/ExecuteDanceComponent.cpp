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

bool ExecuteDanceComponent::start(int argc, char*argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    
    // STRATEGY: Initialize YarpActionPlayer connection instead of CTP service
    yAPClientPortName = "/ExecuteDanceComponent/yarpActionsPlayerClient/rpc";
    
    // Clean up any existing port with same name
    if (yarp::os::Network::exists(yAPClientPortName)) {
        yWarning() << "Port" << yAPClientPortName << "already exists, attempting cleanup";
        // Force disconnect any existing connections
        yarp::os::Network::disconnect(yAPClientPortName, "/yarpActionsPlayer/rpc");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    bool b = m_yAPClientPort.open(yAPClientPortName);
    if (!b)
    {
        yError() << "Cannot open yarpActionsPlayer client port";
        return false;
    }
    yarp::os::Network::connect(yAPClientPortName, "/yarpActionsPlayer/rpc");
    
    m_node = rclcpp::Node::make_shared("ExecuteDanceComponentNode");
    m_executeDanceService = m_node->create_service<execute_dance_interfaces::srv::ExecuteDance>("/ExecuteDanceComponent/ExecuteDance",  
                                                                                std::bind(&ExecuteDanceComponent::ExecuteDance,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_isDancingService = m_node->create_service<execute_dance_interfaces::srv::IsDancing>("/ExecuteDanceComponent/IsDancing",
                                                                                std::bind(&ExecuteDanceComponent::IsDancing,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));

    // 1) subscribe to /amcl_pose
    m_amclSub = m_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&ExecuteDanceComponent::amclPoseCallback, this, std::placeholders::_1));

    // 2) load POI coordinates from JSON
    // m_poiCoords: Map storing Point of Interest (POI) coordinates for UC3 tour guidance
    // - Key (std::string): POI name (e.g., "sala_delle_guardie", "madama_start") 
    // - Value (std::pair<double, double>): X,Y coordinates in global map frame
    //   used for robot orientation calculations before executing movements
    // 
    // Data loaded from "conf/board_coords.json" at startup
    // Essential for robot positioning strategy during guided museum tours
    m_poiCoords = loadPoiCoordinates("/home/user1/UC3/conf/board_coords.json");

    // 3) load artwork coordinates from JSON
    // m_artworkCoords: Map storing 3D coordinates of artworks for precise pointing
    // - Key (std::string): Artwork name (e.g., "quadro_1", "quadro_principale")
    // - Value (std::vector<double>): [X, Y, Z] coordinates in global map frame
    //   used for robot arm pointing
    // Data loaded from "conf/artwork_coords.json" at startup
    // Used by ExecutePointingMovement() for Cartesian control commands
    // This allows the robot to accurately point to artworks during tours   
    m_artworkCoords = loadArtworkCoordinates("/home/user1/UC3/conf/artwork_coords.json");

    // 4) open YARP CartesianController client
    {
        yarp::os::Property opts; // Options for CartesianControllerClient
        opts.put("device","cartesiancontrollerclient"); // Device type
        opts.put("local", "/ExecuteDanceComponent/cartesian:o"); // Local port name
        opts.put("remote","/cartesianController/rpc:i"); // Remote port to connect to
        
        RCLCPP_INFO(m_node->get_logger(), "Attempting to connect to CartesianController...");
        
        if (!m_cartesianClient.open(opts)) {
            RCLCPP_ERROR(m_node->get_logger(), 
                        "CRITICAL: Failed to open CartesianController client");
            RCLCPP_ERROR(m_node->get_logger(), 
                        "Please ensure CartesianController server is running at /cartesianController/rpc:i");
            RCLCPP_WARN(m_node->get_logger(), 
                       "ExecuteDanceComponent will continue without CartesianControl (pointing disabled)");
            m_cartesianCtrl = nullptr;
        } else if (m_cartesianClient.isValid()) { 
            m_cartesianClient.view(m_cartesianCtrl); // View the CartesianControl interface
            if (!m_cartesianCtrl) {
                RCLCPP_ERROR(m_node->get_logger(), 
                            "Failed to get CartesianControl interface");
            } else {
                RCLCPP_INFO(m_node->get_logger(), 
                           "Successfully connected to CartesianController");
            }
        }
    }

    RCLCPP_DEBUG(m_node->get_logger(), "ExecuteDanceComponent::start");      
    return true;
}

bool ExecuteDanceComponent::close()
{
    if (m_cartesianClient.isValid()) {
        m_cartesianClient.close();
    }
    
    rclcpp::shutdown();  
    return true;
}

void ExecuteDanceComponent::spin()
{
    rclcpp::spin(m_node);  
}

void ExecuteDanceComponent::executeTask(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request)
{
    // STRATEGY: Robot must orient towards the POI before executing any movement
    // This ensures proper positioning for artwork pointing
    auto it = m_poiCoords.find(request->dance_name);
    if (it != m_poiCoords.end() && m_cartesianCtrl) {
        // Calculate direction towards POI from current robot position
        double dx     = it->second.first  - m_currentX;   // Delta X towards POI
        double dy     = it->second.second - m_currentY;   // Delta Y towards POI
        double goal   = std::atan2(dy, dx);               // Target angle towards POI
        double dtheta = goal - m_currentYaw;              // Angular difference to correct
        
        // Normalize angle to [-π, π] range
        while (dtheta > M_PI)  dtheta -= 2*M_PI;
        while (dtheta < -M_PI) dtheta += 2*M_PI;
        
        RCLCPP_INFO(m_node->get_logger(),
                    "STRATEGY: Orienting towards POI '%s': angular correction=%.2f rad", 
                    request->dance_name.c_str(), dtheta);
        
        // IMPLEMENTATION: Use Cartesian controller to orient robot torso
        // instead of base to avoid navigation issues
        yarp::sig::Vector currentPos(3), currentOri(4);
        if (m_cartesianCtrl->getPose(currentPos, currentOri)) {
            yarp::sig::Vector newOri = currentOri;
            // Apply rotation around Z-axis (yaw)
            newOri[2] = sin(dtheta/2);  // Z component of quaternion
            newOri[3] = cos(dtheta/2);  // W component of quaternion
            
            // Execute synchronous orientation movement
            m_cartesianCtrl->goToPoseSync(currentPos, newOri);
            m_cartesianCtrl->waitMotionDone();
        }
    }
    else {
        RCLCPP_WARN(m_node->get_logger(),
                    "No POI found '%s', skipping orientation.",
                    request->dance_name.c_str());
    }

    bool done_with_getting_dance = false;
    do {
        //calls the GetMovement service
        auto getMovementClientNode = rclcpp::Node::make_shared("ExecuteDanceComponentGetMovementNode");
        std::shared_ptr<rclcpp::Client<dance_interfaces::srv::GetMovement>> getMovementClient =
        getMovementClientNode->create_client<dance_interfaces::srv::GetMovement>("/DanceComponent/GetMovement");
        auto getMovementRequest = std::make_shared<dance_interfaces::srv::GetMovement::Request>();
        
        while(!getMovementClient->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'getMovementClient'. Exiting.");
                return;
            }
        }
        
        auto getMovementResult = getMovementClient->async_send_request(getMovementRequest);
        rclcpp::spin_until_future_complete(getMovementClientNode, getMovementResult);
        auto movement = getMovementResult.get();
        
        if (movement->is_ok == false) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "ExecuteDanceComponent::ExecuteDance. Movement not found, skipping...");
        } else {
            bool status;
            
            if (movement->part_name == "pointing_command") {
                status = ExecutePointingMovement(movement);
            }
            else {
                // STRATEGY: Use YarpActionPlayer instead of CTP service for dance movements
                status = SendMovementToYAP(request->dance_name, 1.0f); // Using part_name as action name
            }
            
            if (!status) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Movement failed to send. Is YarpActionPlayer running?");
                continue;
            }
        }

        //calls the UpdateMovement service
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
        
        auto updateMovementResult = updateMovementClient->async_send_request(updateMovementRequest);
        rclcpp::spin_until_future_complete(updateMovementClientNode, updateMovementResult);
        auto updateMovementResponse = updateMovementResult.get();
        done_with_getting_dance = updateMovementResponse->done_with_dance;
        
    } while (!done_with_getting_dance);
}

void ExecuteDanceComponent::timerTask(float time)
{
    m_timerMutex.lock();
    m_timerTask = true;
    m_timerMutex.unlock();
    
    RCLCPP_INFO_STREAM(m_node->get_logger(), "Start Timer seconds: " << static_cast<int>(time));
    
    std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(time)));

    m_timerMutex.lock();
    m_timerTask = false;
    m_timerMutex.unlock();
    
    RCLCPP_INFO_STREAM(m_node->get_logger(), "End Timer ");
}

void ExecuteDanceComponent::ExecuteDance(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request,
             std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Response> response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::ExecuteDance " << request->dance_name);
    
    // calls the SetDance service
    auto setDanceClientNode = rclcpp::Node::make_shared("ExecuteDanceComponentSetDanceNode");
    std::shared_ptr<rclcpp::Client<dance_interfaces::srv::SetDance>> setDanceClient =
    setDanceClientNode->create_client<dance_interfaces::srv::SetDance>("/DanceComponent/SetDance");
    auto setDanceRequest = std::make_shared<dance_interfaces::srv::SetDance::Request>();
    setDanceRequest->dance = request->dance_name;
    m_danceName = request->dance_name;
    
    while (!setDanceClient->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setDanceClient'. Exiting.");
        }
    }
    
    auto setDanceResult = setDanceClient->async_send_request(setDanceRequest);
    rclcpp::spin_until_future_complete(setDanceClientNode, setDanceResult);
    auto setDanceResponse = setDanceResult.get();
    
    if (setDanceResponse->is_ok != true) {
        RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::ExecuteDance name: " << request->dance_name);
        response->is_ok = false;
        response->error_msg = "Dance not found";
        return;
    }
    
    if (m_threadExecute.joinable()) {
        m_threadExecute.join();
    }
    
    m_threadExecute = std::thread([this, request]() { executeTask(request); });
    
    response->is_ok = true;
}

void ExecuteDanceComponent::IsDancing(const std::shared_ptr<execute_dance_interfaces::srv::IsDancing::Request> request,
             std::shared_ptr<execute_dance_interfaces::srv::IsDancing::Response> response) 
{
    (void)request;
    
    m_timerMutex.lock();
    response->is_dancing = m_timerTask;
    m_timerMutex.unlock();
    
    RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::IsDancing " << response->is_dancing);
    response->is_ok = true;
}

// STRATEGY: YarpActionPlayer methods instead of CTP service
bool ExecuteDanceComponent::SendMovementToYAP(const std::string &actionName, float speedFactor)
{
    yarp::os::Bottle res;
    yarp::os::Bottle cmd;

    cmd.clear();
    res.clear();
    cmd.addString("choose_action");
    cmd.addString(actionName);

    std::cout << "ExecuteDanceComponent::SendMovementToYAP sending bottle content: " << cmd.toString() << " and action name is " << actionName << std::endl;

    bool status = m_yAPClientPort.write(cmd, res);

    if (!status)
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP Failed to send choose_action command to YAP");
        return false;
    }

    if (res.get(0).asVocab32() != yarp::os::createVocab32('o', 'k'))
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP YAP did not accept the action: " << actionName);
        return false;
    }
    else
    {
        RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP YAP accepted the action: " << actionName);
    }

    cmd.clear();
    res.clear();
    cmd.addString("speed_factor");
    cmd.addFloat32(speedFactor);

    std::cout << "ExecuteDanceComponent::SendMovementToYAP sending bottle content: " << cmd.toString() << " and speed factor is " << speedFactor << std::endl;

    status = m_yAPClientPort.write(cmd, res);

    if (!status)
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP Failed to send speed_factor command to YAP");
        return false;
    }

    if (res.get(0).asVocab32() != yarp::os::createVocab32('o', 'k'))
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP YAP did not accept the speed factor: " << speedFactor);
        return false;
    }
    else
    {
        RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP YAP accepted the speed factor: " << speedFactor);
    }

    cmd.clear();
    res.clear();
    cmd.addString("start");

    std::cout << "ExecuteDanceComponent::SendMovementToYAP sending bottle content: " << cmd.toString() << " and action name is " << actionName << std::endl;

    status = m_yAPClientPort.write(cmd, res);

    if (!status)
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP Failed to send start command to YAP");
        return false;
    }

    if (res.get(0).asVocab32() != yarp::os::createVocab32('o', 'k'))
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP YAP did not accept the action: " << actionName);
        return false;
    }
    else
    {
        RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP YAP accepted the action: " << actionName);
    }

    while (m_timerTask)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    cmd.clear();
    res.clear();
    cmd.addString("reset");

    status = m_yAPClientPort.write(cmd, res);

    if (!status)
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP Failed to send reset command to YAP");
        return false;
    }

    RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP Reset status: " << res.get(0).toString());

    cmd.clear();
    res.clear();
    cmd.addString("speed_factor");
    cmd.addFloat32(1.0f);

    std::cout << "ExecuteDanceComponent::SendMovementToYAP sending bottle content: " << cmd.toString() << " and speed factor is " << 1 << std::endl;

    status = m_yAPClientPort.write(cmd, res);

    if (!status)
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP Failed to send speed_factor command to YAP");
        return false;
    }

    if (res.get(0).asVocab32() != yarp::os::createVocab32('o', 'k'))
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP YAP did not accept the speed factor: " << 1);
        return false;
    }
    else
    {
        RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP YAP accepted the speed factor: " << 1);
    }

    return true;
}

void ExecuteDanceComponent::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // STRATEGY: Keep robot pose updated for coordinate transformations
    // This information is CRITICAL for accurate artwork pointing
    
    // Extract position from AMCL message
    m_currentX = msg->pose.pose.position.x;
    m_currentY = msg->pose.pose.position.y;
    
    // STRATEGY: Extract yaw orientation from quaternion
    // Convert quaternion to Euler yaw angle for 2D transformations
    double qx = msg->pose.pose.orientation.x,
           qy = msg->pose.pose.orientation.y,
           qz = msg->pose.pose.orientation.z,
           qw = msg->pose.pose.orientation.w;
    
    // Quaternion to yaw conversion formula
    m_currentYaw = std::atan2(2*(qw*qz + qx*qy),1 - 2*(qy*qy + qz*qz));
}

std::map<std::string, std::pair<double, double>> ExecuteDanceComponent::loadPoiCoordinates(const std::string& filename)
{
    std::map<std::string, std::pair<double, double>> poiMap;
    
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(m_node->get_logger(), "Impossibile aprire il file POI: '%s'", filename.c_str());
            return poiMap;
        }
        
        auto config = nlohmann::ordered_json::parse(file);
        
        for (auto& [name, data] : config.at("boards").items()) {
            double x = data.at("poi").at("x").get<double>();
            double y = data.at("poi").at("y").get<double>();
            poiMap.emplace(name, std::make_pair(x, y));
        }
        
        RCLCPP_INFO(m_node->get_logger(), "Caricate %zu POI da '%s'", poiMap.size(), filename.c_str());
        
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(m_node->get_logger(), "Errore parsing POI file '%s': %s", filename.c_str(), ex.what());
    }
    
    return poiMap;
}

std::map<std::string, std::vector<double>> ExecuteDanceComponent::loadArtworkCoordinates(const std::string& filename)
{
    std::map<std::string, std::vector<double>> artworkMap;
    
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(m_node->get_logger(), "Unable to open artwork coordinates file: '%s'", filename.c_str());
            return artworkMap;
        }
        
        auto config = nlohmann::ordered_json::parse(file);
        
        // STRATEGY: Load complete X,Y,Z coordinates for each artwork
        // Coordinates are in the global museum map reference frame
        for (auto& [name, data] : config.at("artworks").items()) {
            double x = data.at("x").get<double>();  // Absolute X position in map frame
            double y = data.at("y").get<double>();  // Absolute Y position in map frame  
            double z = data.at("z").get<double>();  // Z height of artwork (for precise pointing)
            
            // Store as vector [x, y, z] for use with CartesianControl
            artworkMap.emplace(name, std::vector<double>{x, y, z});
            
            RCLCPP_INFO(m_node->get_logger(), 
                       "Loaded artwork '%s' at coordinates [%.2f, %.2f, %.2f]", 
                       name.c_str(), x, y, z);
        }
        
        RCLCPP_INFO(m_node->get_logger(), "Loaded %zu artworks from '%s'", 
                   artworkMap.size(), filename.c_str());
        
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(m_node->get_logger(), "Error parsing artwork file '%s': %s", 
                    filename.c_str(), ex.what());
    }
    
    return artworkMap;
}

bool ExecuteDanceComponent::ExecutePointingMovement(const std::shared_ptr<dance_interfaces::srv::GetMovement::Response> movement)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), 
                      "POINTING STRATEGY: Starting pointing_command (NOT normal dance movement)");
    
    // STRATEGY: For pointing_command, joints[0] is the artwork index to point to
    // This is DIFFERENT from normal movements where joints contains actual angles
    std::string artworkName = extractArtworkName(movement->joints);
    
    if (artworkName.empty()) {
        RCLCPP_ERROR(m_node->get_logger(), "ERROR: Invalid artwork name for pointing");
        return false;
    }
    
    RCLCPP_INFO_STREAM(m_node->get_logger(), 
                      "STRATEGY: Executing pointing towards artwork: " << artworkName);
    
    // STRATEGY: Lookup absolute coordinates of artwork from loaded database
    auto it = m_artworkCoords.find(artworkName);
    if (it == m_artworkCoords.end()) {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), 
                           "ERROR: Artwork '" << artworkName << "' not found in database");
        return false;
    }
    
    // Absolute coordinates of artwork in map frame
    std::vector<double> mapCoords = it->second;
    RCLCPP_INFO(m_node->get_logger(), 
               "STRATEGY: Artwork coordinates [%.2f, %.2f, %.2f] in map frame", 
               mapCoords[0], mapCoords[1], mapCoords[2]);
    
    // STRATEGY: Transform from map coordinates to robot coordinates
    std::vector<double> robotCoords = transformMapToRobot(mapCoords);
    
    if (robotCoords.empty()) {
        RCLCPP_ERROR(m_node->get_logger(), "ERROR: Coordinate transformation failed");
        return false;
    }
    
    // STRATEGY: Send Cartesian command (NOT joint command like normal dance)
    return sendCartesianCommand(robotCoords);
}

std::string ExecuteDanceComponent::extractArtworkName(const std::vector<float>& joints)
{
    // SPECIAL STRATEGY: For pointing_command, joints[0] contains artwork INDEX
    // NOT joint angles like for normal dance movements
    if (joints.empty()) {
        RCLCPP_ERROR(m_node->get_logger(), "ERROR: Empty joints vector for pointing_command");
        // Fallback to first artwork if no index provided
        if (!m_artworkCoords.empty()) {
            return m_artworkCoords.begin()->first;
        }
        return "";  // Ultimate fallback
    }
    
    // Convert first element from float to integer (artwork index)
    int artworkIndex = static_cast<int>(joints[0]);
    
    RCLCPP_INFO(m_node->get_logger(), 
               "STRATEGY: Requested pointing towards artwork with index %d", artworkIndex);
    
    // STRATEGY: Convert numeric index to artwork string name
    // Create ordered list of artwork names for consistent indexing
    std::vector<std::string> artworkNames;
    for (const auto& pair : m_artworkCoords) {
        artworkNames.push_back(pair.first);
    }
    
    // Validate index range and return corresponding artwork name
    if (artworkIndex >= 0 && artworkIndex < static_cast<int>(artworkNames.size())) {
        std::string artworkName = artworkNames[artworkIndex];
        RCLCPP_INFO(m_node->get_logger(), 
                   "STRATEGY: Index %d corresponds to artwork '%s'", 
                   artworkIndex, artworkName.c_str());
        return artworkName;
    }
    
    // Index out of range - fallback to first artwork
    RCLCPP_ERROR(m_node->get_logger(), 
                "ERROR: Artwork index %d out of range [0-%zu], using fallback", 
                artworkIndex, artworkNames.size()-1);
    
    if (!artworkNames.empty()) {
        return artworkNames[0];
    }
    
    return "quadro_1";  // Ultimate fallback
}

std::vector<double> ExecuteDanceComponent::transformMapToRobot(const std::vector<double>& mapCoords)
{
    // STRATEGY: Transform artwork coordinates from map frame to robot frame
    // Uses current robot pose provided by AMCL for accurate transformation
    
    if (mapCoords.size() < 3) {
        RCLCPP_ERROR(m_node->get_logger(), "ERROR: Incomplete map coordinates");
        return {};
    }
    
    // Absolute coordinates of artwork in map frame
    double artworkX_map = mapCoords[0];
    double artworkY_map = mapCoords[1]; 
    double artworkZ_map = mapCoords[2];
    
    RCLCPP_INFO(m_node->get_logger(), 
               "TRANSFORMATION: Artwork in map frame [%.2f, %.2f, %.2f]", 
               artworkX_map, artworkY_map, artworkZ_map);
    
    RCLCPP_INFO(m_node->get_logger(), 
               "TRANSFORMATION: Robot pose [%.2f, %.2f, %.2f rad]", 
               m_currentX, m_currentY, m_currentYaw);
    
    // STRATEGY: Calculate relative artwork coordinates with respect to robot
    // 2D transformation for X,Y + height adjustment for Z
    
    // Delta in map frame
    double dx_map = artworkX_map - m_currentX;
    double dy_map = artworkY_map - m_currentY;
    
    // Inverse rotation to transform from map to robot frame
    double cos_theta = std::cos(-m_currentYaw);  // Inverse rotation matrix
    double sin_theta = std::sin(-m_currentYaw);
    
    // Apply 2D rotation transformation
    double x_robot = dx_map * cos_theta - dy_map * sin_theta;
    double y_robot = dx_map * sin_theta + dy_map * cos_theta;
    double z_robot = artworkZ_map - 1.0;  // Adjust for robot base height (1m offset)
    
    RCLCPP_INFO(m_node->get_logger(), 
               "TRANSFORMATION: Artwork in robot frame [%.2f, %.2f, %.2f]", 
               x_robot, y_robot, z_robot);
    
    return {x_robot, y_robot, z_robot};
}

bool ExecuteDanceComponent::sendCartesianCommand(const std::vector<double>& coords)
{
    // STRATEGY: Use YARP CartesianControl instead of CTP service (used for dance)
    if (!m_cartesianCtrl) {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ERROR: Cartesian controller not initialized");
        return false;
    }
    
    if (coords.size() < 3) {
        RCLCPP_ERROR(m_node->get_logger(), "ERROR: Incomplete coordinates for Cartesian command");
        return false;
    }
    
    // STRATEGY: Prepare command for 3D pointing
    yarp::sig::Vector targetPos(3);
    targetPos[0] = coords[0];  // X relative to robot
    targetPos[1] = coords[1];  // Y relative to robot  
    targetPos[2] = coords[2];  // Z adjusted for robot height
    
    // STRATEGY: Set pointing orientation (end-effector pointing forward)
    yarp::sig::Vector targetOri(4);  // Quaternion [x, y, z, w]
    targetOri[0] = 0.0;  // No rotation around X
    targetOri[1] = 0.0;  // No rotation around Y
    targetOri[2] = 0.0;  // No rotation around Z
    targetOri[3] = 1.0;  // Identity quaternion (no rotation)
    
    RCLCPP_INFO(m_node->get_logger(), 
               "CARTESIAN COMMAND: Pointing towards [%.2f, %.2f, %.2f]", 
               coords[0], coords[1], coords[2]);
    
    // STRATEGY: Send synchronous command to YARP Cartesian controller
    // This will move the robot's end-effector towards target coordinates
    bool success = m_cartesianCtrl->goToPoseSync(targetPos, targetOri);
    
    if (success) {
        // Wait for motion completion before proceeding
        m_cartesianCtrl->waitMotionDone();
        RCLCPP_INFO_STREAM(m_node->get_logger(), "SUCCESS: Pointing command executed successfully");
        return true;
    } else {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ERROR: Failed to send pointing command");
        return false;
    }
}
