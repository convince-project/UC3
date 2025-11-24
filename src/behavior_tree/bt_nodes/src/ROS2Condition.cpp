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

#include <chrono>         // std::chrono::seconds

#include <ROS2Condition.h>
#define VERBOSE_LOGGING
ROS2Condition::ROS2Condition(const std::string name, const BT::NodeConfiguration& config) :
        ConditionNode(name, config)
{
    BT::Optional<std::string> is_monitored = BT::TreeNode::getInput<std::string>("isMonitored");
    if (is_monitored.value() == "true")
    {
        m_suffixMonitor = "_mon";
    }

    
    BT::Optional<std::string> interface = BT::TreeNode::getInput<std::string>("interface");
    bool ok = init();
    if(!ok)
    {
       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Something went wrong in the node init() of %s", ConditionNode::name().c_str());
    }
}

BT::PortsList ROS2Condition::providedPorts()
{
    return { BT::InputPort<std::string>("interface"),
             BT::InputPort<std::string>("isMonitored") };
}


bool ROS2Condition::init()
{

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }

    m_node = rclcpp::Node::make_shared(ConditionNode::name()+ "ConditionLeaf");
    m_clientTick = m_node->create_client<bt_interfaces_dummy::srv::TickCondition>(ConditionNode::name() + "Skill/tick" + m_suffixMonitor);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"name -- " << ConditionNode::name() << " -- suffixmonitor " << m_suffixMonitor);
    
    return true;

}


bool ROS2Condition::stop()
{
    rclcpp::shutdown();
    return 0;
}



int ROS2Condition::sendTickToSkill() 
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    auto msg = bt_interfaces_dummy::msg::ConditionResponse();
    auto request = std::make_shared<bt_interfaces_dummy::srv::TickCondition::Request>();
    while (!m_clientTick->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service TickCondition. Exiting.");
        return msg.SKILL_FAILURE;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s service TickCondition not available, waiting again...", ConditionNode::name().c_str());
    }
    auto result = m_clientTick->async_send_request(request);
    // std::this_thread::sleep_for (std::chrono::milliseconds(100));
    if (rclcpp::spin_until_future_complete(m_node, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        return result.get()->status;
    }
    return msg.SKILL_FAILURE;
}


BT::NodeStatus ROS2Condition::tick()
{
    auto message = bt_interfaces_dummy::msg::ConditionResponse();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node %s sending tick to skill", ConditionNode::name().c_str());
    auto time_start = std::chrono::high_resolution_clock::now();
    auto status = sendTickToSkill();
    auto time_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start);
    #ifdef VERBOSE_LOGGING
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node %s tick to skill took %ld milliseconds", ConditionNode::name().c_str(), duration.count());
    m_tick_count++;
    if (m_tick_count > 4)
    {
        m_average_time = (duration.count() + (m_tick_count - 1-4) * m_average_time) / (m_tick_count-4);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node %s average tick time %f, number of ticks %d", ConditionNode::name().c_str(), m_average_time, m_tick_count);
    }
    #endif
    switch (status) {
        case message.SKILL_SUCCESS:
            return BT::NodeStatus::SUCCESS;
        case message.SKILL_FAILURE:
            return BT::NodeStatus::FAILURE;
        default:
            break;
    }
    return BT::NodeStatus::FAILURE;
}
