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
#include <dance_interfaces/srv/get_dance.hpp>
#include <dance_interfaces/srv/get_dance_duration.hpp>
#include <dance_interfaces/srv/get_movement.hpp>
#include <dance_interfaces/srv/set_dance.hpp>
#include <dance_interfaces/srv/get_part_names.hpp>
#include <dance_interfaces/srv/update_movement.hpp>
#include <dance_interfaces/srv/get_best_dance.hpp>
#include <execute_dance_interfaces/srv/execute_dance.hpp>
#include <execute_dance_interfaces/srv/reset_dance.hpp>
// #include <execute_dance_interfaces/srv/is_dancing.hpp>


class ExecuteDanceComponent 
{
public:
    ExecuteDanceComponent() = default;
    bool start(int argc, char*argv[]);
    bool close();
    void spin();
    //open a service for all the execute_dance_interfaces
    void ExecuteDance(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request,
                      std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Response> response);
    // void IsDancing(const std::shared_ptr<execute_dance_interfaces::srv::IsDancing::Request> request,
    //                std::shared_ptr<execute_dance_interfaces::srv::IsDancing::Response> response);
    void ResetDance(const std::shared_ptr<execute_dance_interfaces::srv::ResetDance::Request> request,
                   std::shared_ptr<execute_dance_interfaces::srv::ResetDance::Response> response);
private:
    
    bool SendMovementToYAP(const std::string &actionName, float speedFactor);
 
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<execute_dance_interfaces::srv::ExecuteDance>::SharedPtr m_executeDanceService;
    rclcpp::Service<execute_dance_interfaces::srv::ResetDance>::SharedPtr m_resetDanceService;
    // rclcpp::Service<execute_dance_interfaces::srv::IsDancing>::SharedPtr m_isDancingService;

    // yarpActionsPlayers client port name
    std::string yAPClientPortName;

    // yarpActionsPlayers client port
    yarp::os::Port m_yAPClientPort;

    bool m_timerTask{false};

    std::string m_lastExecutedDanceCategory{""};

    std::shared_ptr<rclcpp::Node> getBestDanceClientNode;
    std::shared_ptr<rclcpp::Client<dance_interfaces::srv::GetBestDance>> getBestDanceClient;
};
