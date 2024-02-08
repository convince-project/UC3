/******************************************************************************
 *                                                                            *
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
/**
 * @file ROS2Node.cpp
 * @authors: Stefano Bernagozzi <stefano.bernagozzi@iit.it>
 */

#include "ROS2Node.h"
#include <behaviortree_cpp_v3/leaf_node.h>
#include <iostream>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

using namespace std;
using namespace BT;

ROS2Node::ROS2Node(const std::string& name, const NodeConfiguration& config)
{

}

// PortsList ROS2Node::providedPorts()
// {
//     return { InputPort<std::string>("nodeName") };
//     return { InputPort<std::string>("topicName") };
//     return { InputPort<std::string>("interface") };
// }

bool ROS2Node::init()
{

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }

    m_node = rclcpp::Node::make_shared(m_name);
    m_client = m_node->create_client<bt_interfaces::srv::RequestAck>(m_topicName + "/RequestAck" + m_suffixMonitor);
    m_clientStart = m_node->create_client<bt_interfaces::srv::SendStart>(m_topicName + "/SendStart" + m_suffixMonitor);
    m_clientStop = m_node->create_client<bt_interfaces::srv::SendStop>(m_topicName + "/SendStop" + m_suffixMonitor);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"name " << m_name << "topicname " << m_topicName << "suffixmonitor " << m_suffixMonitor);
    
    return true;

}


bool ROS2Node::stop()
{
    rclcpp::shutdown();
    return 0;
}


int ROS2Node::requestAck()
{
    auto message = bt_interfaces::msg::RequestAck();
    int status;
    do {
        status = requestStatus();
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
    } while(status == message.SKILL_IDLE);
    return status;
}


bool ROS2Node::sendStart() {
    std::lock_guard<std::mutex> lock(m_requestMutex);
    auto requestStart = std::make_shared<bt_interfaces::srv::SendStart::Request>();
    while (!m_clientStart->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service start. Exiting.");
        return false;
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service start not available, waiting again..." << m_name);
    }
    auto resultStart = m_clientStart->async_send_request(requestStart);

    std::this_thread::sleep_for (std::chrono::milliseconds(100));
    if (rclcpp::spin_until_future_complete(m_node, resultStart) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        return true;
    }
    return false;
}


bool ROS2Node::sendStop() {
    std::lock_guard<std::mutex> lock(m_requestMutex);
    auto requestStop = std::make_shared<bt_interfaces::srv::SendStop::Request>();
    while (!m_clientStop->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service start. Exiting.");
        return false;
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service start not available, waiting again..." << m_name);
    }
    auto resultStop = m_clientStop->async_send_request(requestStop);

    std::this_thread::sleep_for (std::chrono::milliseconds(100));
    if (rclcpp::spin_until_future_complete(m_node, resultStop) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        return true;
    }
    return false;
}


NodeStatus ROS2Node::tick() //this need to be overwritten
{
    return NodeStatus::FAILURE;
}


int ROS2Node::requestStatus() 
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    auto msg = bt_interfaces::msg::RequestAck();
    auto request = std::make_shared<bt_interfaces::srv::RequestAck::Request>();
    while (!m_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service RequestAck. Exiting.");
        return msg.SKILL_FAILURE;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service RequestAck not available, waiting again...");
    }
    auto result = m_client->async_send_request(request);
    std::this_thread::sleep_for (std::chrono::milliseconds(100));
    if (rclcpp::spin_until_future_complete(m_node, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        return result.get()->status.status;
    }
    return msg.SKILL_FAILURE;
}

