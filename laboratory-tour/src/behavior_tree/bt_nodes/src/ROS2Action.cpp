/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
/**
 * @file ROS2Action.cpp
 * @authors: Stefano Bernagozzi <stefano.bernagozzi@iit.it>
 */

#include <chrono>         // std::chrono::seconds

#include <ROS2Action.h>

ROS2Action::ROS2Action(const std::string name, const BT::NodeConfiguration& config) :
        ActionNodeBase(name, config)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "name %s", this->name().c_str());
    BT::Optional<std::string> node_name = BT::TreeNode::getInput<std::string>("nodeName");
    if (node_name)
    {
        m_name = node_name.value();
    }

    BT::Optional<std::string> is_monitored = BT::TreeNode::getInput<std::string>("isMonitored");
    if (is_monitored.value() == "true")
    {
        m_suffixMonitor = "_mon";
    }
    BT::Optional<std::string> interface = BT::TreeNode::getInput<std::string>("interface");
    bool ok = init();

    if(!ok)
    {
       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Something went wrong in the node init() of %s", name.c_str());
    }
}


int ROS2Action::sendTickToSkill() 
{
    auto msg = bt_interfaces::msg::ActionResponse();
    auto request = std::make_shared<bt_interfaces::srv::TickAction::Request>();
    while (!m_clientTick->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service TickAction. Exiting.");
        return msg.SKILL_FAILURE;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service TickAction not available, waiting again...");
    }
    auto result = m_clientTick->async_send_request(request);
    std::this_thread::sleep_for (std::chrono::milliseconds(100));
    if (rclcpp::spin_until_future_complete(m_node, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        return result.get()->status.status;
    }
    return msg.SKILL_FAILURE;
}


BT::NodeStatus ROS2Action::tick()
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    auto message = bt_interfaces::msg::ActionResponse();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node %s sending tick to skill", ActionNodeBase::name().c_str());
    auto status = sendTickToSkill();
    switch (status) {
        case message.SKILL_RUNNING:
            return BT::NodeStatus::RUNNING;
        case message.SKILL_SUCCESS:
            return BT::NodeStatus::SUCCESS;
        case message.SKILL_FAILURE:
            return BT::NodeStatus::FAILURE;
        default:
            break;
    }
    return BT::NodeStatus::FAILURE;
}



BT::PortsList ROS2Action::providedPorts()
{
    return { BT::InputPort<std::string>("nodeName"), 
             BT::InputPort<std::string>("interface"),
             BT::InputPort<std::string>("isMonitored")  };
}

void ROS2Action::halt()
{
    bool success = false;
    do {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node %s sending halt to skill", ActionNodeBase::name().c_str());
        auto request = std::make_shared<bt_interfaces::srv::HaltAction::Request>();
        while (!m_clientHalt->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service TickAction. Exiting.");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service TickAction not available, waiting again...");
        }
        auto result = m_clientHalt->async_send_request(request);
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
        if (rclcpp::spin_until_future_complete(m_node, result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            success = true;
        }
    } while (!success);
}



bool ROS2Action::init()
{

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }

    m_node = rclcpp::Node::make_shared(m_name+ "Leaf");
    m_clientTick = m_node->create_client<bt_interfaces::srv::TickAction>(m_name + "Skill/tick" + m_suffixMonitor);
    m_clientHalt = m_node->create_client<bt_interfaces::srv::HaltAction>(m_name + "Skill/halt" + m_suffixMonitor);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"name " << m_name << "suffixmonitor " << m_suffixMonitor);
    
    return true;

}


bool ROS2Action::stop()
{
    rclcpp::shutdown();
    return 0;
}
