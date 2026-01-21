/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "ExecuteAudioComponent.h"

bool ExecuteAudioComponent::start(int argc, char *argv[])
{
    if (!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }

    yAPClientPortName = "/ExecuteAudioComponent/yarpAudioPlayerClient/rpc";
    bool b = m_yAPClientPort.open(yAPClientPortName);
    if (!b)
    {
        yError() << "Cannot open yarpAudioPlayer client port";
        return false;
    }
    yarp::os::Network::connect(yAPClientPortName, "/yarpAudioPlayer/rpc");

    m_node = rclcpp::Node::make_shared("ExecuteAudioComponentNode");


    m_executeAudioService = m_node->create_service<execute_dance_interfaces::srv::ExecuteAudio>("/ExecuteAudioComponent/ExecuteAudio",
                                                                                                std::bind(&ExecuteAudioComponent::ExecuteAudio,
                                                                                                          this,
                                                                                                          std::placeholders::_1,
                                                                                                          std::placeholders::_2));


    RCLCPP_DEBUG(m_node->get_logger(), "ExecuteAudioComponent::start");
    return true;
}

bool ExecuteAudioComponent::ConfigureYARP(yarp::os::ResourceFinder &rf)
{
    // ---------------------YARP NETWORK----------------------------
    string m_audioConfigFilePath = "/home/user1/UC3/conf/audio/configuration.json";
    if (rf.check("AUDIO-CONFIGURATION"))
    {
        yarp::os::Searchable &component_config = rf.findGroup("AUDIO-CONFIGURATION");
        if (component_config.check("filepath"))
        {
            m_audioConfigFilePath = component_config.find("filepath").asString();
        }
    }

    std::ifstream f(m_audioConfigFilePath);
    m_audioConfig = nlohmann::json::parse(f);

    m_speechToTextPort.open(m_speechToTextClientName);
    // Try Automatic port connection
    if (!yarp::os::Network::connect(m_speechToTextServerName, m_speechToTextClientName))
    {
        yWarning() << "[DialogComponent::start] Unable to connect to: " << m_speechToTextServerName;
    }
}

bool ExecuteAudioComponent::close()
{

    rclcpp::shutdown();
    return true;
}

void ExecuteAudioComponent::spin()
{
    rclcpp::spin(m_node);
}

void ExecuteAudioComponent::ExecuteAudio(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteAudio::Request> request,
                                         std::shared_ptr<execute_dance_interfaces::srv::ExecuteAudio::Response> response)
{

    float speechTime = request->speech_time;
    std::string danceCategory = request->dance_name;
    std::string danceName;
    bool isSpeedFactorOk = false;
    float speedFactor = 1.0;
    float danceDuration = 0.0;
    

    if (speechTime > 0.0f)
    {
        // call the GetBestDance service
        
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

bool ExecuteAudioComponent::PlayAudioFile(const std::string &actionName, float speedFactor)
{
    yarp::os::Bottle res;
    yarp::os::Bottle cmd;

    cmd.clear();
    res.clear();
    cmd.addString("reset");

    bool status = m_yAPClientPort.write(cmd, res);

    if (!status)
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteAudioComponent::SendMovementToYAP Failed to send reset command to YAP");
        return false;
    }

    RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteAudioComponent::SendMovementToYAP Reset status: " << res.get(0).toString());


    cmd.clear();
    res.clear();
    cmd.addString("choose_action");
    cmd.addString(actionName);

    std::cout << "ExecuteAudioComponent::SendMovementToYAP sending bottle content: " << cmd.toString() << " and action name is " << actionName << std::endl;

    status = m_yAPClientPort.write(cmd, res);

    if (!status)
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteAudioComponent::SendMovementToYAP Failed to send choose_action command to YAP");
        return false;
    }

    if (res.get(0).asVocab32() != yarp::os::createVocab32('o', 'k'))
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteAudioComponent::SendMovementToYAP YAP did not accept the action: " << actionName);
        return false;
    }
    else
    {
        RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteAudioComponent::SendMovementToYAP YAP accepted the action: " << actionName);
    }

    cmd.clear();
    res.clear();
    cmd.addString("speed_factor");
    cmd.addFloat32(speedFactor);

    std::cout << "ExecuteAudioComponent::SendMovementToYAP sending bottle content: " << cmd.toString() << " and speed factor is " << speedFactor << std::endl;

    status = m_yAPClientPort.write(cmd, res);

    if (!status)
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteAudioComponent::SendMovementToYAP Failed to send speed_factor command to YAP");
        return false;
    }

    if (res.get(0).asVocab32() != yarp::os::createVocab32('o', 'k'))
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteAudioComponent::SendMovementToYAP YAP did not accept the speed factor: " << speedFactor);
        return false;
    }
    else
    {
        RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteAudioComponent::SendMovementToYAP YAP accepted the speed factor: " << speedFactor);
    }

    cmd.clear();
    res.clear();
    cmd.addString("start");

    std::cout << "ExecuteAudioComponent::SendMovementToYAP sending bottle content: " << cmd.toString() << " and action name is " << actionName << std::endl;

    status = m_yAPClientPort.write(cmd, res);

    if (!status)
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteAudioComponent::SendMovementToYAP Failed to send start command to YAP");
        return false;
    }

    if (res.get(0).asVocab32() != yarp::os::createVocab32('o', 'k'))
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "ExecuteAudioComponent::SendMovementToYAP YAP did not accept the action: " << actionName);
        return false;
    }
    else
    {
        RCLCPP_INFO_STREAM(m_node->get_logger(), "ExecuteAudioComponent::SendMovementToYAP YAP accepted the action: " << actionName);
    }

    return true;
}