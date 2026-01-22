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

    m_node = rclcpp::Node::make_shared("ExecuteAudioComponentNode");


    m_executeAudioService = m_node->create_service<execute_audio_interfaces::srv::ExecuteAudio>("/ExecuteAudioComponent/ExecuteAudio",
                                                                                                std::bind(&ExecuteAudioComponent::ExecuteAudio,
                                                                                                          this,
                                                                                                          std::placeholders::_1,
                                                                                                          std::placeholders::_2));

    RCLCPP_DEBUG(m_node->get_logger(), "ExecuteAudioComponent::start");
    return true;
}

bool ExecuteAudioComponent::ConfigureYARP(yarp::os::ResourceFinder &rf)
{
    // ---------------------YARP AUDIO CONFIGURATION----------------------------
    std::string m_audioConfigFilePath = "/home/user1/UC3/conf/audio_configuration.json";
    if (rf.check("AUDIO-CONFIGURATION"))
    {
        yarp::os::Searchable &audio_config = rf.findGroup("AUDIO-CONFIGURATION");
        if (audio_config.check("filepath"))
        {
            m_audioConfigFilePath = audio_config.find("filepath").asString();
        }
    }

    std::cout << "reading audio configuration file from " << m_audioConfigFilePath << std::endl;

    std::ifstream file(m_audioConfigFilePath);
    
    if (!file.is_open())
    {
        yError() << "Failed to open file!";
        return false;
    }

    // Read the entire file into a string
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    if (content.empty())
    {
        yError() << "File content is empty!";
        return false;
    }

    yInfo() << "Raw file content:\n" << content;

    try
    {
        m_audioConfig = nlohmann::json::parse(content);
    }
    catch (const nlohmann::json::parse_error &e)
    {
        yError() << "JSON parse error:" << e.what();
        return false;
    }

    yInfo() << "Audio configuration loaded successfully:\n" << m_audioConfig.dump(4);


    // YARP AUDIO PLAYER
    std::string localAudioPlayerClientName = "/ExecuteAudioComponentYarpAudioPlayerClient/audio:o";
    std::string remoteAudioPlayerServerName = "/yarpAudioPlayer/rpc";
    if (rf.check("YARP-AUDIO-PLAYER"))
    {
        yarp::os::Searchable &component_config = rf.findGroup("YARP-AUDIO-PLAYER");
        if (component_config.check("localAudioPlayerClientName"))
        {
            localAudioPlayerClientName = component_config.find("localAudioPlayerClientName").asString();
        }
        if (component_config.check("remoteAudioPlayerServerName"))
        {
            remoteAudioPlayerServerName = component_config.find("remoteAudioPlayerServerName").asString();
        }
    }

    if (!m_yAPClientPort.open(localAudioPlayerClientName))
    {
        yError() << "Cannot open yarpAudioPlayer client port";
        return false;
    }
    if (!yarp::os::Network::connect(localAudioPlayerClientName, remoteAudioPlayerServerName)) {
        yWarning() << "[ExecuteAudioComponent::configureYarp] Unable to connect to: " << remoteAudioPlayerServerName;
    }

    return true;
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

void ExecuteAudioComponent::ExecuteAudio(const std::shared_ptr<execute_audio_interfaces::srv::ExecuteAudio::Request> request,
                                         std::shared_ptr<execute_audio_interfaces::srv::ExecuteAudio::Response> response)
{

    // float audioTime = request->audio_time;
    std::string audioName = request->audio_name;

    std::string audioFileName = m_audioConfig[audioName];

    bool status;

    std::cout << "AudioAudioComponent::ExecuteAudio sending audio: " << audioName << std::endl;
    status = PlayAudioFile(audioFileName);
    if (!status)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Audio failed to send to YarpAudioPlayer");
        response->is_ok = false;
        return;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Done sending Audio to Player");
    response->is_ok = true;
}

bool ExecuteAudioComponent::PlayAudioFile(const std::string &audioName)
{
    yarp::os::Bottle res;
    yarp::os::Bottle cmd;

    cmd.clear();
    res.clear();
    cmd.addString("play");
    cmd.addString(audioName);

    bool status = m_yAPClientPort.write(cmd, res);

    if (!status)
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "[ExecuteAudioComponent::SendMovementToYAP] Failed to send play command to YAP");
        return false;
    }

    RCLCPP_INFO_STREAM(m_node->get_logger(), "[ExecuteAudioComponent::SendMovementToYAP] play status: " << res.get(0).toString());

    return true;
}