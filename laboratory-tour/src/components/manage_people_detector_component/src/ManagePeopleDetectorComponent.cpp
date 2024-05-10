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
    // m_client_get_state = m_node->create_client<lifecycle_msgs::srv::GetState>("/dr_spaam_ros_local/get_state");
    // m_client_change_state = m_node->create_client<lifecycle_msgs::srv::ChangeState>("/dr_spaam_ros_local/change_state");

    RCLCPP_INFO(m_node->get_logger(), "ManagePeopleDetectorComponent::start");
    return true;

}



std::optional<lifecycle_msgs::msg::State> ManagePeopleDetectorComponent::get_state(const std::chrono::seconds& timeout) {


    // calls the service to get the state of the managed node
    auto getStateClientNode = rclcpp::Node::make_shared("ManagePeopleDetectorGetStateNode");
    auto getStateClient = getStateClientNode->create_client<lifecycle_msgs::srv::GetState>("/dr_spaam_ros_local/get_state");
    auto getStateRequest = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    while (!getStateClient->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'getStateClient'. Exiting.");
        }
    }
    auto getStateResult = getStateClient->async_send_request(getStateRequest);
    auto futureGetStateResult = rclcpp::spin_until_future_complete(getStateClientNode, getStateResult);
    if (futureGetStateResult == rclcpp::FutureReturnCode::SUCCESS)
    {
        return getStateResult.get()->current_state;
    }
    else
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "[DialogComponent::InterpretCommand] getStateClient Failed" << __LINE__);
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
    // calls the change state service to activate the managed node
    auto startPeopleDetectorRequestClientNode = rclcpp::Node::make_shared("ManagePeopleDetectorStartNode");
    auto startPeopleDetectorRequestClient = startPeopleDetectorRequestClientNode->create_client<lifecycle_msgs::srv::ChangeState>("/dr_spaam_ros_local/change_state");
    auto startPeopleDetectorRequest = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    startPeopleDetectorRequest->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    while (!startPeopleDetectorRequestClient->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'startPeopleDetectorRequestClient'. Exiting.");
        }
    }
    auto startPeopleDetectorResult = startPeopleDetectorRequestClient->async_send_request(startPeopleDetectorRequest);
    auto futureStartPeopleDetectorResult = rclcpp::spin_until_future_complete(startPeopleDetectorRequestClientNode, startPeopleDetectorResult);
    if (futureStartPeopleDetectorResult == rclcpp::FutureReturnCode::SUCCESS)
    {
        response->is_ok = true;
    }
    else
    {
        response->is_ok = false;
    }
}


void ManagePeopleDetectorComponent::StopPeopleDetector([[maybe_unused]] const std::shared_ptr<manage_service_interfaces::srv::StopService::Request> request,
             std::shared_ptr<manage_service_interfaces::srv::StopService::Response>      response) 
{
    // calls the change state service to deactivate the managed node
    auto stopPeopleDetectorRequestClientNode = rclcpp::Node::make_shared("ManagePeopleDetectorStopNode");
    auto stopPeopleDetectorRequestClient = stopPeopleDetectorRequestClientNode->create_client<lifecycle_msgs::srv::ChangeState>("/dr_spaam_ros_local/change_state");
    auto stopPeopleDetectorRequest = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    stopPeopleDetectorRequest->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
    while (!stopPeopleDetectorRequestClient->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'stopPeopleDetectorRequestClient'. Exiting.");
        }
    }
    auto stopPeopleDetectorResult = stopPeopleDetectorRequestClient->async_send_request(stopPeopleDetectorRequest);
    auto futureStopPeopleDetectorResult = rclcpp::spin_until_future_complete(stopPeopleDetectorRequestClientNode, stopPeopleDetectorResult);
    if (futureStopPeopleDetectorResult == rclcpp::FutureReturnCode::SUCCESS)
    {
        response->is_ok = true;
    }
    else
    {
        response->is_ok = false;
    }

}