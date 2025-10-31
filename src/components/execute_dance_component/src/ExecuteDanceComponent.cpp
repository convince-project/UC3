/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "ExecuteDanceComponent.h"

bool ExecuteDanceComponent::start(int argc, char *argv[])
{
    if (!rclcpp::ok())
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

    m_resetDanceService = m_node->create_service<execute_dance_interfaces::srv::ResetDance>("/ExecuteDanceComponent/ResetDance",
                                                                                            std::bind(&ExecuteDanceComponent::ResetDance,
                                                                                                      this,
                                                                                                      std::placeholders::_1,
                                                                                                      std::placeholders::_2));
    // m_isDancingService = m_node->create_service<execute_dance_interfaces::srv::IsDancing>("/ExecuteDanceComponent/IsDancing",
    //                                                                                       std::bind(&ExecuteDanceComponent::IsDancing,
    //                                                                                                 this,
    //                                                                                                 std::placeholders::_1,
    //                                                                                                 std::placeholders::_2));

    RCLCPP_DEBUG(m_node->get_logger(), "ExecuteDanceComponent::start");
    return true;
}

bool ExecuteDanceComponent::close()
{

    rclcpp::shutdown();
    return true;
}

void ExecuteDanceComponent::spin()
{
    rclcpp::spin(m_node);
}

void ExecuteDanceComponent::ExecuteDance(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request,
                                         std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Response> response)
{
    // RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::ExecuteDance " << request->dance_name);
    // // calls the SetDance service
    // auto setDanceClientNode = rclcpp::Node::make_shared("ExecuteDanceComponentSetDanceNode");
    // std::shared_ptr<rclcpp::Client<dance_interfaces::srv::SetDance>> setDanceClient =
    //     setDanceClientNode->create_client<dance_interfaces::srv::SetDance>("/DanceComponent/SetDance");
    // auto setDanceRequest = std::make_shared<dance_interfaces::srv::SetDance::Request>();
    // setDanceRequest->dance = request->dance_name;
    // m_danceName = request->dance_name;
    // while (!setDanceClient->wait_for_service(std::chrono::seconds(1)))
    // {
    //     if (!rclcpp::ok())
    //     {
    //         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setDanceClient'. Exiting.");
    //     }
    // }
    // auto setDanceResult = setDanceClient->async_send_request(setDanceRequest);
    // auto futureSetDanceResult = rclcpp::spin_until_future_complete(setDanceClientNode, setDanceResult);
    // auto setDanceResponse = setDanceResult.get();
    // if (setDanceResponse->is_ok != true)
    // {
    //     RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::ExecuteDance name: " << request->dance_name);
    //     response->is_ok = false;
    //     response->error_msg = "Dance not found";
    //     return;
    // }

    float speechTime = request->speech_time;
    std::string danceCategory = request->dance_name;
    std::string danceName;
    bool isSpeedFactorOk = false;
    float speedFactor = 1.0;
    float danceDuration = 0.0;
    

    if (speechTime > 0.0f)
    {
        // call the GetBestDance service
        auto getBestDanceClientNode = rclcpp::Node::make_shared("ExecuteDanceComponentGetBestDanceNode");
        std::shared_ptr<rclcpp::Client<dance_interfaces::srv::GetBestDance>> getBestDanceClient =
            getBestDanceClientNode->create_client<dance_interfaces::srv::GetBestDance>("/DanceComponent/GetBestDance");
        auto getBestDanceRequest = std::make_shared<dance_interfaces::srv::GetBestDance::Request>();
        while (!getBestDanceClient->wait_for_service(std::chrono::milliseconds(100)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'getBestDanceClient'. Exiting.");
            }
        }
        getBestDanceRequest->dance_category = danceCategory;
        getBestDanceRequest->speech_duration = speechTime;
        auto getBestDanceResult = getBestDanceClient->async_send_request(getBestDanceRequest);
        auto futureGetBestDanceResult = rclcpp::spin_until_future_complete(getBestDanceClientNode, getBestDanceResult);
        auto bestDance = getBestDanceResult.get();

        if (bestDance->is_ok == false)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "ExecuteDanceComponent::ExecuteDance. Dance  not found");
            response->is_ok = false;
            return;
        }

        // assume reaching the starting position will take 5 secs
        float reaching_start_position_time = 5.0;
        // if the last executed dance category is the same of the current one, then we can assume that
        // the last ending position is the same of the current start position, and estimate a lower
        // reaching-start-position time
        if (m_lastExecutedDanceCategory == danceCategory)
        {
            RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::ExecuteDance. Dance is the same as last executed");
            reaching_start_position_time = 2.0; // seconds
        }

        float speech_dance_synchronization_speed_factor = (bestDance->dance_duration + reaching_start_position_time) / speechTime;

        std::cout << "Best dance selected : " << bestDance->dance_name
                  << " with duration: " << bestDance->dance_duration
                  << ", with reaching start position time: " << reaching_start_position_time
                  << ", with speech time: " << speechTime
                  << " and speed factor: " << speech_dance_synchronization_speed_factor << std::endl;

        danceName = bestDance->dance_name;
        isSpeedFactorOk = (speech_dance_synchronization_speed_factor > 0.75) && (speech_dance_synchronization_speed_factor < 1.5);

        speedFactor = speech_dance_synchronization_speed_factor;
        danceDuration = bestDance->dance_duration;

        RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::ExecuteDance Duration: " << danceDuration << " taking into account reaching start position time: " << reaching_start_position_time);
        
    }
    else
    {
        danceName = danceCategory;
        isSpeedFactorOk = true;
        speedFactor = 1.0;
    }

    std::cout << "Is speed factor ok?" << isSpeedFactorOk << std::endl;

    if (isSpeedFactorOk)
    {
        bool status;

        std::cout << "ExecuteDanceComponent::executeTask sending dance: " << danceName << " with speed factor: " << speedFactor << std::endl;
        status = SendMovementToYAP(danceName, speedFactor);
        if (!status)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Movement failed to send to YAP");
            response->is_ok = false;
            return;
        }
        m_lastExecutedDanceCategory = danceCategory;
    }
    else
    {   
        std::cout << "ExecuteDanceComponent::executeTask not sending dance because speed factor too low or too high. Speed Factor: " << speedFactor << std::endl;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Done getting Dance");
    response->is_ok = true;
}

// void ExecuteDanceComponent::IsDancing(const std::shared_ptr<execute_dance_interfaces::srv::IsDancing::Request> request,
//                                       std::shared_ptr<execute_dance_interfaces::srv::IsDancing::Response> response)
// {
//     m_timerMutex.lock();
//     response->is_dancing = m_timerTask;
//     m_timerMutex.unlock();
//     RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::IsDancing " << response->is_dancing);
//     response->is_ok = true;
// }

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

    return true;
}

void ExecuteDanceComponent::ResetDance(const std::shared_ptr<execute_dance_interfaces::srv::ResetDance::Request> request,
                                       std::shared_ptr<execute_dance_interfaces::srv::ResetDance::Response> response)
{
    yarp::os::Bottle cmd;
    yarp::os::Bottle res;

    cmd.clear();
    res.clear();
    cmd.addString("speed_factor");
    cmd.addFloat32(1.0f);

    std::cout << "ExecuteDanceComponent::SendMovementToYAP sending bottle content: " << cmd.toString() << " and speed factor is " << 1 << std::endl;

    bool status = m_yAPClientPort.write(cmd, res);

    if (!status)
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP Failed to send speed_factor command to YAP");
        return;
    }

    if (res.get(0).asVocab32() != yarp::os::createVocab32('o', 'k'))
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP YAP did not accept the speed factor: " << 1);
        return;
    }
    else
    {
        RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::SendMovementToYAP YAP accepted the speed factor: " << 1);
    }

    cmd.clear();
    res.clear();
    cmd.addString("reset");

    std::cout << "ExecuteDanceComponent::StopDance sending bottle content: " << cmd.toString() << std::endl;

    status = m_yAPClientPort.write(cmd, res);

    if (!status)
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteDanceComponent::StopDance Failed to send stop command to YAP");
        response->is_ok = false;
        response->error_msg = "Failed to send stop command to YAP";
        return;
    }

    if (res.get(0).asVocab32() != yarp::os::createVocab32('o', 'k'))
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteDanceComponent::StopDance YAP did not accept the stop command");
        response->is_ok = false;
        response->error_msg = "YAP did not accept the stop command";
        return;
    }
    else
    {
        RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteDanceComponent::StopDance YAP accepted the stop command");
    }

    response->is_ok = true;
}