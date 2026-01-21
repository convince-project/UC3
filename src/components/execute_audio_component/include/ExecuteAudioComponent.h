/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <text_to_speech_interfaces/srv/is_speaking.hpp>
#include <nlohmann/json.hpp>


class ExecuteAudioComponent 
{
public:
    ExecuteAudioComponent() = default;
    bool start(int argc, char*argv[]);
    bool close();
    void spin();

    bool ConfigureYARP(yarp::os::ResourceFinder &rf);

    //open a service for all the execute_dance_interfaces
    void ExecuteAudio(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteAudio::Request> request,
                      std::shared_ptr<execute_dance_interfaces::srv::ExecuteAudio::Response> response);
private:
    
    bool SendMovementToYAP(const std::string &actionName, float speedFactor);
 
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<execute_dance_interfaces::srv::ExecuteAudio>::SharedPtr m_executeAudioService;

    // yarpActionsPlayers client port name
    std::string yAPClientPortName;

    // yarpActionsPlayers client port
    yarp::os::Port m_yAPClientPort;

    nlohmann::json m_audioConfig;
    
};
