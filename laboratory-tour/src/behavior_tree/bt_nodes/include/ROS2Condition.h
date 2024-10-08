/******************************************************************************
 *                                                                            *
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
/**
 * @file ROS2Condition.h
 * @authors: Stefano Bernagozzi <stefano.bernagozzi@iit.it>
 */


#pragma once

#include <bt_interfaces_dummy/msg/condition_response.hpp>
#include <mutex>
#include <bt_interfaces_dummy/srv/tick_condition.hpp>
#include <string>
#include<behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>

class ROS2Condition :  public BT::ConditionNode
{
public:
    ROS2Condition(const std::string name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    int sendTickToSkill();
    static BT::PortsList providedPorts();
    bool init();
    bool stop();

private:
    std::mutex m_requestMutex;
    rclcpp::Client<bt_interfaces_dummy::srv::TickCondition>::SharedPtr m_clientTick;
    std::shared_ptr<rclcpp::Node> m_node;
    std::string m_name;
    std::string m_suffixMonitor;
};
