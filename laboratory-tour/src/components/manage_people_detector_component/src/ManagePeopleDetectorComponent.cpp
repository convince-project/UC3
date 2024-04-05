/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "ManagePeopleDetectorComponent.h"

bool ManagePeopleDetectorComponent::start(int argc, char*argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("ManagePeopleDetectorComponentNode");
    m_startPeopleDetectorService = m_node->create_service<manage_service_interfaces::srv::StartService>("/ManagePeopleDetectorComponent/StartPeopleDetector",  
                                                                                std::bind(&ManagePeopleDetectorComponent::StartPeopleDetector,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_stopPeopleDetectorService = m_node->create_service<manage_service_interfaces::srv::StopService>("/ManagePeopleDetectorComponent/StopPeopleDetector",  
                                                                                std::bind(&ManagePeopleDetectorComponent::StopPeopleDetector,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
                                                                                    m_node = rclcpp::Node::make_shared(m_name);
    m_client_get_state = m_node->create_client<lifecycle_msgs::srv::GetState>("/dr_spaam_ros_node/get_state");
    m_client_change_state = m_node->create_client<lifecycle_msgs::srv::ChangeState>("/dr_spaam_ros_node/change_state");

    RCLCPP_INFO(m_node->get_logger(), "ManagePeopleDetectorComponent::start");
    return true;

}

bool ManagePeopleDetectorComponent::is_client_available(const std::chrono::seconds& timeout) {

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

std::optional<lifecycle_msgs::msg::State> ManagePeopleDetectorComponent::get_state(const std::chrono::seconds& timeout) {

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



bool ManagePeopleDetectorComponent::close()
{
    rclcpp::shutdown();  
    return true;
}

void ManagePeopleDetectorComponent::spin()
{
    rclcpp::spin(m_node);  
}

void ManagePeopleDetectorComponent::StartPeopleDetector([[maybe_unused]] const std::shared_ptr<manage_service_interfaces::srv::StartService::Request> request,
             std::shared_ptr<manage_service_interfaces::srv::StartService::Response>      response) 
{
    auto internalRequest = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    internalRequest->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    
    auto internalResult = m_client_change_state->async_send_request(internalRequest);
    
    if (rclcpp::spin_until_future_complete(m_node,internalResult,m_timeout) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_DEBUG(m_node->get_logger(), "StartServicesDataModel::start(). Success");
        response->is_ok = true;    
    } else {
        RCLCPP_DEBUG(m_node->get_logger(), "StartServicesDataModel::start(). Failure");
        response->is_ok = false;
    }
}


void ManagePeopleDetectorComponent::StopPeopleDetector([[maybe_unused]] const std::shared_ptr<manage_service_interfaces::srv::StopService::Request> request,
             std::shared_ptr<manage_service_interfaces::srv::StopService::Response>      response) 
{
    auto internalRequest = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    internalRequest->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
    
    auto internalResult = m_client_change_state->async_send_request(internalRequest);
    
    if (rclcpp::spin_until_future_complete(m_node,internalResult,m_timeout) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_DEBUG(m_node->get_logger(), "StopServicesDataModel::stop(). Success");
        response->is_ok = true;    
    } else {
        RCLCPP_DEBUG(m_node->get_logger(), "StopServicesDataModel::stop(). Failure");
        response->is_ok = false;
    }
}

