/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "SchedulerComponent.h"

bool SchedulerComponent::start(int argc, char*argv[])
{
    if (argc >= 2)
    {
        m_tourStorage = std::make_shared<TourStorage>(); // Loads the tour json from the file and saves a reference to the class.
        if( !m_tourStorage->LoadTour(argv[1], argv[2]))
        {
            return false;
        }
    }
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("SchedulerComponentNode");
    m_updatePoiService = m_node->create_service<scheduler_interfaces::srv::UpdatePoi>("/SchedulerComponent/UpdatePoi",  
                                                                                std::bind(&SchedulerComponent::UpdatePoi,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_resetService = m_node->create_service<scheduler_interfaces::srv::Reset>("/SchedulerComponent/Reset",  
                                                                                std::bind(&SchedulerComponent::Reset,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getCurrentPoiService = m_node->create_service<scheduler_interfaces::srv::GetCurrentPoi>("/SchedulerComponent/GetCurrentPoi",  
                                                                                std::bind(&SchedulerComponent::GetCurrentPoi,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    RCLCPP_DEBUG(m_node->get_logger(), "SchedulerComponent::start");
    std::cout << "SchedulerComponent::start";        
    return true;

}

bool SchedulerComponent::close()
{
    rclcpp::shutdown();  
    return true;
}

void SchedulerComponent::spin()
{
    rclcpp::spin(m_node);  
}

void SchedulerComponent::Reset([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::Reset::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::Reset::Response>      response) 
{
    m_currentPoi = 0;
    response->is_ok = true;
}


void SchedulerComponent::UpdatePoi([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::UpdatePoi::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::UpdatePoi::Response>      response) 
{
    m_currentPoi = (m_currentPoi + 1) % m_tourStorage->GetTour().getPoIsList().size();
    response->is_ok = true;
}


void SchedulerComponent::GetCurrentPoi([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetCurrentPoi::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::GetCurrentPoi::Response>      response) 
{
    response->poi_name = m_tourStorage->GetTour().getPoIsList()[m_currentPoi];
    response->poi_number = m_currentPoi;
    response->is_ok = true;
}
