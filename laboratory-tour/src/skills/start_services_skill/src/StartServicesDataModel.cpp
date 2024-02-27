/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
#include "StartServicesDataModel.h"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include <QDebug>
#include <QTimer>
#include <QScxmlStateMachine>



void StartServicesDataModel::set_name(std::string name)
{
    m_name=name; 
}

bool StartServicesDataModel::setup(const QVariantMap &initialDataValues)
{
    Q_UNUSED(initialDataValues)

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }

    m_node = rclcpp::Node::make_shared(m_name);
    m_client_get_state = m_node->create_client<lifecycle_msgs::srv::GetState>("/dr_spaam_ros_node/get_state");
    m_client_change_state = m_node->create_client<lifecycle_msgs::srv::ChangeState>("/dr_spaam_ros_node/change_state");

    RCLCPP_DEBUG(m_node->get_logger(), "StartServicesDataModel::start");
    std::cout << "StartServicesDataModel::start";

    return true;
}

bool StartServicesDataModel::is_client_available(const std::chrono::seconds& timeout) {

    // Check if client is available, else return
    bool is_client_available = m_client_get_state->wait_for_service(timeout);
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return false;
    }
    if(!is_client_available){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, aborting.");
        return false;
    }

    return true;
}

std::optional<lifecycle_msgs::msg::State> StartServicesDataModel::get_state(const std::chrono::seconds& timeout) {

    if(!is_client_available(timeout)) {
        return std::nullopt;
    }

    // Get the state of the managed node
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto result = m_client_get_state->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(m_node, result,timeout) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_DEBUG(m_node->get_logger(), "StartServicesDataModel::get_state(). Success");
        return result.get()->current_state;
    } else {
        RCLCPP_DEBUG(m_node->get_logger(), "StartServicesDataModel::get_state(). Failure");
        return std::nullopt;
    }

}

bool StartServicesDataModel::start() {

    // if(!is_client_available()){
    //     return false;
    // }

    // // Check if managed node is in state active and make it inactive
    // // Check https://github.com/ros2/rcl_interfaces/blob/humble/lifecycle_msgs/msg/State.msg for reference
    // std::optional<lifecycle_msgs::msg::State> current_state = this->get_state();
    // if(!current_state) {
    //     return false;
    // }

    // if(current_state.value().id != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    //     RCLCPP_ERROR(m_node->get_logger(), "Cannot start service since it is not inactive");
    //     return false;
    // }
    // else {

        // Change the state of the managed node to inactive
        // Check https://github.com/ros2/rcl_interfaces/blob/humble/lifecycle_msgs/msg/Transition.msg for reference
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
        
        auto result = m_client_change_state->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(m_node,result,m_timeout) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_DEBUG(m_node->get_logger(), "StartServicesDataModel::start(). Success");
            return true;
        } else {
            RCLCPP_DEBUG(m_node->get_logger(), "StartServicesDataModel::start(). Failure");
            return false;
        }
    // }

    // return true;
}