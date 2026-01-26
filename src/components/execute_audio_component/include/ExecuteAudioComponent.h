/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
# pragma once

#include <rclcpp/rclcpp.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <nlohmann/json.hpp>
#include <fstream>

// ExecuteAudio Interfaces
#include <execute_audio_interfaces/srv/execute_audio.hpp>


class ExecuteAudioComponent 
{
public:
    ExecuteAudioComponent() = default;
    bool start(int argc, char*argv[]);
    bool close();
    void spin();

    bool ConfigureYARP(yarp::os::ResourceFinder &rf);

    //open a service for all the execute_dance_interfaces
    void ExecuteAudio(const std::shared_ptr<execute_audio_interfaces::srv::ExecuteAudio::Request> request,
                      std::shared_ptr<execute_audio_interfaces::srv::ExecuteAudio::Response> response);
private:
    
    bool PlayAudioFile(const std::string &audioName);
 
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<execute_audio_interfaces::srv::ExecuteAudio>::SharedPtr m_executeAudioService;

    // yarpActionsPlayers client port
    yarp::os::Port m_yAPClientPort;

    nlohmann::json m_audioConfig;
    
};
