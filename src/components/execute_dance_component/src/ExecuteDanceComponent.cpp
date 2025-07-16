/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "ExecuteDanceComponent.h"


bool ExecuteDanceComponent::start(int argc, char*argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }

    yAPClientPortName = "/ExecuteDanceComponent/yarpActionsPlayerClient/rpc";
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
    bool done_with_getting_dance = false;
    //call the GetDanceDuration service
    auto getDanceDurationClientNode = rclcpp::Node::make_shared("ExecuteDanceComponentGetDanceDurationNode");
    std::shared_ptr<rclcpp::Client<dance_interfaces::srv::GetDanceDuration>> getDanceDurationClient =
        getDanceDurationClientNode->create_client<dance_interfaces::srv::GetDanceDuration>("/DanceComponent/GetDanceDuration");
    auto getDanceDurationRequest = std::make_shared<dance_interfaces::srv::GetDanceDuration::Request>();
    while (!getDanceDurationClient->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'getDanceDurationClient'. Exiting.");
        }
    }
    auto getDanceDurationResult = getDanceDurationClient->async_send_request(getDanceDurationRequest);
    auto futureGetDanceDurationResult = rclcpp::spin_until_future_complete(getDanceDurationClientNode, getDanceDurationResult);
    auto danceDuration = getDanceDurationResult.get();
    
    bool status;

    float speech_time = request->speech_time;

    float speech_dance_synchronization_speed_factor = danceDuration->duration / speech_time;

    if (danceDuration->is_ok == false) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "ExecuteDanceComponent::ExecuteDance. Dance duration not found");
    }
    else
    {
        RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::ExecuteDance Duration: " << danceDuration->duration << " taking into account reaching start position time: " << reaching_start_position_time);
        if (m_threadTimer.joinable()) {
            m_threadTimer.join();
            RCLCPP_INFO_STREAM(m_node->get_logger(), "Timer task joined ");
        }
        m_threadTimer = std::thread([this, speech_time]() { timerTask(speech_time); });
    }

    std::cout << "Dance duration: " << danceDuration->duration << " and speech time: " << request->speech_time << std::endl;
    std::cout << "ExecuteDanceComponent::executeTask sending dance: " << request->dance_name << " with speed factor: " << speech_dance_synchronization_speed_factor << std::endl;

    status = SendMovementToYAP(request->dance_name, speech_dance_synchronization_speed_factor);
    if (!status)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Movement failed to send to YAP");
        return;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Done getting Dance");
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
    auto futureSetDanceResult = rclcpp::spin_until_future_complete(setDanceClientNode, setDanceResult);
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
             std::shared_ptr<execute_dance_interfaces::srv::IsDancing::Response>      response) 
{
    m_timerMutex.lock();
    response->is_dancing = m_timerTask;
    m_timerMutex.unlock();
    RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::IsDancing " << response->is_dancing);
    response->is_ok = true;
}


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
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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