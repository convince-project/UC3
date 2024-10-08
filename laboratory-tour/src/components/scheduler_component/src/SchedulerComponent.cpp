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
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error loading tour");
            return false;
        }
    }
    else
    {
        std::cerr << "Error: file path is missing" << std::endl;
        return false;
    }
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("SchedulerComponentNode");
    m_setPoiService = m_node->create_service<scheduler_interfaces_dummy::srv::SetPoi>("/SchedulerComponent/SetPoi",
                                                                                std::bind(&SchedulerComponent::SetPoi,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getCurrentPoiService = m_node->create_service<scheduler_interfaces_dummy::srv::GetCurrentPoi>("/SchedulerComponent/GetCurrentPoi",
                                                                                std::bind(&SchedulerComponent::GetCurrentPoi,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));


    RCLCPP_DEBUG(m_node->get_logger(), "SchedulerComponent::start");
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



void SchedulerComponent::SetPoi([[maybe_unused]] const std::shared_ptr<scheduler_interfaces_dummy::srv::SetPoi::Request> request,
             std::shared_ptr<scheduler_interfaces_dummy::srv::SetPoi::Response>      response)
{
    RCLCPP_INFO(m_node->get_logger(), "SchedulerComponent::SetPoi %d",  request->poi_number);
    m_currentPoi = (request->poi_number) % m_tourStorage->GetTour().getPoIsList().size();
    response->is_ok = true;
    std::string text = "Update Poi to: " + std::to_string(m_currentPoi) + " - " + m_tourStorage->GetTour().getPoIsList()[m_currentPoi];
}


void SchedulerComponent::GetCurrentPoi([[maybe_unused]] const std::shared_ptr<scheduler_interfaces_dummy::srv::GetCurrentPoi::Request> request,
             std::shared_ptr<scheduler_interfaces_dummy::srv::GetCurrentPoi::Response>      response)
{
    // response->poi_name = m_tourStorage->GetTour().getPoIsList()[m_currentPoi];
    response->poi_number = m_currentPoi;
    RCLCPP_INFO(m_node->get_logger(), "SchedulerComponent::GetCurrentPoi number: %s", response->poi_number.c_str());
    response->is_ok = true;
}

