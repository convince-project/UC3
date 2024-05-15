/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "AllowedToMoveComponent.h"

bool AllowedToMoveComponent::start(int argc, char*argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_allowedToMove.store(false);
    m_node = rclcpp::Node::make_shared("AllowedToMoveComponentNode");
    m_setAllowedToMoveService = m_node->create_service<allowed_to_move_interfaces::srv::SetAllowedToMove>("/AllowedToMoveComponent/SetAllowedToMove",  
                                                                                std::bind(&AllowedToMoveComponent::SetAllowedToMove,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_isAllowedToMoveService = m_node->create_service<allowed_to_move_interfaces::srv::IsAllowedToMove>("/AllowedToMoveComponent/IsAllowedToMove",
                                                                                std::bind(&AllowedToMoveComponent::IsAllowedToMove,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    RCLCPP_INFO(m_node->get_logger(), "AllowedToMoveComponent::start");
    return true;

}

bool AllowedToMoveComponent::close()
{
    rclcpp::shutdown();  
    return true;
}

void AllowedToMoveComponent::spin()
{
    rclcpp::spin(m_node);  
}

void AllowedToMoveComponent::IsAllowedToMove([[maybe_unused]] const std::shared_ptr<allowed_to_move_interfaces::srv::IsAllowedToMove::Request> request,
                std::shared_ptr<allowed_to_move_interfaces::srv::IsAllowedToMove::Response>      response) 
{
    response->is_allowed_to_move = m_allowedToMove.load();
    response->is_ok = true;
}

void AllowedToMoveComponent::SetAllowedToMove([[maybe_unused]] const std::shared_ptr<allowed_to_move_interfaces::srv::SetAllowedToMove::Request> request,
                std::shared_ptr<allowed_to_move_interfaces::srv::SetAllowedToMove::Response>      response) 
{
    m_allowedToMove.store(request->is_allowed_to_move);
    response->is_ok = true;
}