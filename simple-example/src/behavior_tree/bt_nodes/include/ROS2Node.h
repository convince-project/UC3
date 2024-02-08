/******************************************************************************
 *                                                                            *
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
/**
 * @file ROS2Node.h
 * @authors: Stefano Bernagozzi <stefano.bernagozzi@iit.it>
 */


#pragma once
#include <rclcpp/rclcpp.hpp>
#include <bt_interfaces/msg/request_ack.hpp>
#include <mutex>
#include <bt_interfaces/srv/request_ack.hpp>
#include <bt_interfaces/srv/send_start.hpp>
#include <bt_interfaces/srv/send_stop.hpp>
#include <behaviortree_cpp_v3/action_node.h>

using namespace std;
using namespace BT;

class ROS2Node //: public LeafNode
{
public:
    ROS2Node(const std::string& name, const NodeConfiguration& config);
    NodeStatus tick();
    int requestAck();
    bool sendStart();
    bool sendStop();
    bool init();
    bool stop();
    // static PortsList providedPorts();



protected:
    std::shared_ptr<rclcpp::Node> m_node;
    string m_topicName;
    string m_name;
    string m_suffixMonitor;
    // mutable int8_t m_bt_request; // mutable because status() is const

private:
    int requestStatus();
    std::mutex m_requestMutex;
    rclcpp::Client<bt_interfaces::srv::RequestAck>::SharedPtr m_client;
    rclcpp::Client<bt_interfaces::srv::SendStart>::SharedPtr m_clientStart;
    rclcpp::Client<bt_interfaces::srv::SendStop>::SharedPtr m_clientStop;

};
