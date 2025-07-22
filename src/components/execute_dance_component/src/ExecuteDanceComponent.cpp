/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "ExecuteDanceComponent.h"
#include <fstream>
#include <nlohmann/json.hpp>


bool ExecuteDanceComponent::start(int argc, char*argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    // Ctp Service
    // calls the GetPartNames service
    auto getPartNamesClientNode = rclcpp::Node::make_shared("ExecuteDanceComponentGetPartNamesNode");
    std::shared_ptr<rclcpp::Client<dance_interfaces::srv::GetPartNames>> getPartNamesClient =
        getPartNamesClientNode->create_client<dance_interfaces::srv::GetPartNames>("/DanceComponent/GetPartNames");
    auto getPartNamesRequest = std::make_shared<dance_interfaces::srv::GetPartNames::Request>();
    while (!getPartNamesClient->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'getPartNamesClient'. Exiting.");
        }
    }
    auto getPartNamesResult = getPartNamesClient->async_send_request(getPartNamesRequest);
    // Fix: Remove unused variable warning
    rclcpp::spin_until_future_complete(getPartNamesClientNode, getPartNamesResult);
    auto ctpServiceParts = getPartNamesResult.get()->parts;
    if (!ctpServiceParts.empty())
    {
        for (std::string part : ctpServiceParts)
        {
            yarp::os::Port *ctpPort = new yarp::os::Port;
            std::string portName = "/ExecuteDanceComponentCtpServiceClient/" + part + "/rpc";
            bool b = ctpPort->open(portName);
            if (!b)
            {
                yError() << "Cannot open" << part << " ctpService port";
                return false;
            }
            m_pCtpService.insert({part, *ctpPort});
            yarp::os::Network::connect(portName, "/ctpservice/" + part + "/rpc");
        }
    }
    else
    {
        yWarning() << "Movement part names are empty. No movements will be executed!";
    }
    
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

    // 1) sottoscrivo /amcl_pose
    m_amclSub = m_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&ExecuteDanceComponent::amclPoseCallback, this, std::placeholders::_1));

    // 2) carico le coordinate dei POI dal JSON
    m_poiCoords = loadPoiCoordinates("conf/board_coords.json");

    // 3) apro il client CartesianController di YARP
    {
        yarp::os::Property opts;
        opts.put("device","cartesiancontrollerclient");
        opts.put("local", "/ExecuteDanceComponent/cartesian:o");
        opts.put("remote","/cartesianController/rpc:i");
        m_cartesianClient.open(opts);
        if (m_cartesianClient.isValid()) {
            m_cartesianClient.view(m_cartesianCtrl);
        }
    }

    RCLCPP_DEBUG(m_node->get_logger(), "ExecuteDanceComponent::start");      
    return true;

}

bool ExecuteDanceComponent::close()
{
    for (auto port : m_pCtpService)
    {
        delete &port.second;
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
   
    auto it = m_poiCoords.find(request->dance_name);
    if (it != m_poiCoords.end() && m_cartesianCtrl) {
        double dx     = it->second.first  - m_currentX;
        double dy     = it->second.second - m_currentY;
        double goal   = std::atan2(dy, dx);
        double dtheta = goal - m_currentYaw;
        while (dtheta > M_PI)  dtheta -= 2*M_PI;
        while (dtheta < -M_PI) dtheta += 2*M_PI;
        
        RCLCPP_INFO(m_node->get_logger(),
                    "Orientamento verso POI '%s': dθ=%.2f rad",
                    request->dance_name.c_str(), dtheta);
        
        yarp::sig::Vector currentPos(3), currentOri(4);
        if (m_cartesianCtrl->getPose(currentPos, currentOri)) {
            yarp::sig::Vector newOri = currentOri;
            newOri[2] = sin(dtheta/2);
            newOri[3] = dtheta;
            
            m_cartesianCtrl->goToPoseSync(currentPos, newOri);
            m_cartesianCtrl->waitMotionDone();
        }
    }
    else {
        RCLCPP_WARN(m_node->get_logger(),
                    "Nessuna POI trovata per '%s', salto orientamento",
                    request->dance_name.c_str());
    }

    bool done_with_getting_dance = false;
    do {
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
            if(m_danceName == "idleMove" || m_danceName == "navigationPosition") {
                // Fix: Remove dereference operator since m_pCtpService.at() returns a reference to yarp::os::Port
                status = SendMovementNow(movement->time, movement->offset, movement->joints, m_pCtpService.at(movement->part_name));
            } else {
                status = SendMovementToQueue(movement->time, movement->offset, movement->joints, m_pCtpService.at(movement->part_name));
            }
            
            if (!status) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Movement failed to sent. Is ctpService for part" << movement->part_name << "running?");
                continue;
            }
        }

        // Update movement call
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
             std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Response>      response) 
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
    // Fix: Remove unused variable warning  
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

// Fix: Use (void) to suppress unused parameter warning instead of [[maybe_unused]]
void ExecuteDanceComponent::IsDancing(const std::shared_ptr<execute_dance_interfaces::srv::IsDancing::Request> request,
             std::shared_ptr<execute_dance_interfaces::srv::IsDancing::Response> response) 
{
    (void)request;  // Suppress unused parameter warning
    m_timerMutex.lock();
    response->is_dancing = m_timerTask;
    m_timerMutex.unlock();
    RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::IsDancing " << response->is_dancing);
    response->is_ok = true;
}

bool ExecuteDanceComponent::SendMovementToQueue(float time, int offset, std::vector<float> joints, yarp::os::Port &port)
{
    yarp::os::Bottle res;
    yarp::os::Bottle cmd;
    cmd.addVocab32("ctpq");
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

bool ExecuteDanceComponent::SendMovementNow(float time, int offset, std::vector<float> joints, yarp::os::Port &port)
{
    yarp::os::Bottle res;
    yarp::os::Bottle cmd;
    cmd.addVocab32("ctpn");
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

void ExecuteDanceComponent::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    m_currentX   = msg->pose.pose.position.x;
    m_currentY   = msg->pose.pose.position.y;
    // estraggo yaw da quaternion
    double qx=msg->pose.pose.orientation.x,
           qy=msg->pose.pose.orientation.y,
           qz=msg->pose.pose.orientation.z,
           qw=msg->pose.pose.orientation.w;
    m_currentYaw = std::atan2(2*(qw*qz + qx*qy),
                              1 - 2*(qy*qy + qz*qz));
}

std::map<std::string, std::pair<double, double>> ExecuteDanceComponent::loadPoiCoordinates(const std::string& filename)
{
    std::map<std::string, std::pair<double, double>> poiMap;
    
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(m_node->get_logger(), "Impossibile aprire il file POI: %s", filename.c_str());
            return poiMap;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string name;
            double x, y;
            
            if (iss >> name >> x >> y) {
                poiMap[name] = std::make_pair(x, y);
            }
        }
        
        RCLCPP_INFO(m_node->get_logger(), "Caricate %zu POI da '%s'", poiMap.size(), filename.c_str());
        
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(m_node->get_logger(), "Errore nella lettura del file POI '%s': %s", filename.c_str(), ex.what());
    }
    
    return poiMap;
}
