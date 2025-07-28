/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "DanceComponent.h"

bool DanceComponent::start(int argc, char*argv[])
{
    if (argc >= 2)
    {
        m_movementStorage = std::make_shared<MovementStorage>(); // Loads the movements json from the file and saves a reference to the class.
        if( !m_movementStorage->LoadMovements(argv[1]))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error loading movements file");
            return false;
        }
    }
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("DanceComponentNode");
    m_getMovementService = m_node->create_service<dance_interfaces::srv::GetMovement>("/DanceComponent/GetMovement",  
                                                                                std::bind(&DanceComponent::GetMovement,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_updateMovementService = m_node->create_service<dance_interfaces::srv::UpdateMovement>("/DanceComponent/UpdateMovement",  
                                                                                std::bind(&DanceComponent::UpdateMovement,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_setDanceService = m_node->create_service<dance_interfaces::srv::SetDance>("/DanceComponent/SetDance",  
                                                                                std::bind(&DanceComponent::SetDance,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getDanceService = m_node->create_service<dance_interfaces::srv::GetDance>("/DanceComponent/GetDance",  
                                                                                std::bind(&DanceComponent::GetDance,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getDanceDurationService = m_node->create_service<dance_interfaces::srv::GetDanceDuration>("/DanceComponent/GetDanceDuration",  
                                                                                std::bind(&DanceComponent::GetDanceDuration,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getPartNamesService = m_node->create_service<dance_interfaces::srv::GetPartNames>("/DanceComponent/GetPartNames",
                                                                                std::bind(&DanceComponent::GetPartNames,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));

    RCLCPP_DEBUG(m_node->get_logger(), "DanceComponent::start"); 
    return true;

}

bool DanceComponent::close()
{
    rclcpp::shutdown();  
    return true;
}

void DanceComponent::spin()
{
    rclcpp::spin(m_node);  
}

void DanceComponent::GetMovement([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::GetMovement::Request> request,
             std::shared_ptr<dance_interfaces::srv::GetMovement::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "DanceComponent::GetMovement " );
    Dance dance;
    if(!m_movementStorage->GetMovementsContainer().GetDance(m_currentDance, dance))
    {
        response->is_ok = false;
        response->error_msg = "Dance not found";
        return;
    }   
    response->part_name = dance.GetMovements()[m_currentMovement].GetPartName();
    response->time = dance.GetMovements()[m_currentMovement].GetTime();
    response->offset = dance.GetMovements()[m_currentMovement].GetOffset();
    response->joints = dance.GetMovements()[m_currentMovement].GetJoints();
    response->is_ok = true;
}

void DanceComponent::UpdateMovement([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::UpdateMovement::Request> request,
             std::shared_ptr<dance_interfaces::srv::UpdateMovement::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "DanceComponent::UpdateMovement " );
    Dance dance;
    if(!m_movementStorage->GetMovementsContainer().GetDance(m_currentDance, dance))
    {
        response->is_ok = false;
        response->error_msg = "Dance not found";
        return;
    }
    std::vector<Movement> movements_vec = dance.GetMovements();
    m_currentMovement = (m_currentMovement + 1);
    response->done_with_dance = false;
    if(static_cast<std::size_t>(m_currentMovement) >= movements_vec.size())
    {
        response->done_with_dance = true;
        m_currentMovement = m_currentMovement % movements_vec.size();
    }
    response->is_ok = true;
}

void DanceComponent::SetDance([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::SetDance::Request> request,
             std::shared_ptr<dance_interfaces::srv::SetDance::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "DanceComponent::SetDance name: " << request->dance );
     if(request->dance.empty())
    {
        response->is_ok = false;
        response->error_msg = "Empty dance field";
        return;
    }
    Dance dance;
    if(!m_movementStorage->GetMovementsContainer().GetDance(request->dance, dance))
    {
        response->is_ok = false;
        response->error_msg = "Dance not found";
        return;
    }
    m_currentMovement = 0;
    m_currentDance = request->dance;
    response->is_ok = true;
}

void DanceComponent::GetDance([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::GetDance::Request> request,
             std::shared_ptr<dance_interfaces::srv::GetDance::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "DanceComponent::GetDance name: " << m_currentDance );
    response->dance = m_currentDance;
    response->is_ok = true;
}

void DanceComponent::GetDanceDuration([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::GetDanceDuration::Request> request,
             std::shared_ptr<dance_interfaces::srv::GetDanceDuration::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "DanceComponent::GetDanceDuration" );
    Dance dance;
    if(!m_movementStorage->GetMovementsContainer().GetDance(m_currentDance, dance))
    {
        response->is_ok = false;
        response->error_msg = "Dance not found";
        return;
    }
    response->duration = dance.GetDuration();
    response->is_ok = true;
}


void DanceComponent::GetPartNames([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::GetPartNames::Request> request,
             std::shared_ptr<dance_interfaces::srv::GetPartNames::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "DanceComponent::GetPartNames" );
    std::set<std::string> part_names = m_movementStorage->GetMovementsContainer().GetPartNames();
    for(auto part_name : part_names)
    {
        response->parts.push_back(part_name);
    }
    response->is_ok = true;
}