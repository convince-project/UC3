/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "TimerCheckForPeopleComponent.h"

bool TimerCheckForPeopleComponent::start(int argc, char*argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_timerActive = std::make_shared<std::atomic<bool>>();
    *m_timerActive = false;
    m_node = rclcpp::Node::make_shared("TimerCheckForPeopleComponentNode");
    m_startTimerService = m_node->create_service<timer_check_for_people_interfaces::srv::StartTimer>("/TimerCheckForPeopleComponent/StartTimer",  
                                                                                std::bind(&TimerCheckForPeopleComponent::StartTimer,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_isTimerActiveService = m_node->create_service<timer_check_for_people_interfaces::srv::IsTimerActive>("/TimerCheckForPeopleComponent/IsTimerActive",  
                                                                                std::bind(&TimerCheckForPeopleComponent::IsTimerActive,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    RCLCPP_INFO(m_node->get_logger(), "TimerCheckForPeopleComponent::start");
    return true;

}

bool TimerCheckForPeopleComponent::close()
{
    rclcpp::shutdown();  
    return true;
}

void TimerCheckForPeopleComponent::spin()
{
    rclcpp::spin(m_node);  
}

void TimerCheckForPeopleComponent::IsTimerActive([[maybe_unused]] const std::shared_ptr<timer_check_for_people_interfaces::srv::IsTimerActive::Request> request,
                std::shared_ptr<timer_check_for_people_interfaces::srv::IsTimerActive::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutex);
    response->is_active = *m_timerActive;
    response->is_ok = true;
}

void TimerCheckForPeopleComponent::StartTimer([[maybe_unused]] const std::shared_ptr<timer_check_for_people_interfaces::srv::StartTimer::Request> request,
                std::shared_ptr<timer_check_for_people_interfaces::srv::StartTimer::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (*m_timerActive) {
        response->is_ok = false;
        response->error_msg = "Timer already active";
        return;
    }
    if(m_threadTimer != nullptr && m_threadTimer->joinable())
    {
        m_threadTimer->join();
    }

    m_threadTimer = std::make_shared<std::thread>(Timer,m_timerActive);
    response->is_ok = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------- timer started ------------------------------");
}



void TimerCheckForPeopleComponent::Timer(std::shared_ptr<std::atomic<bool>> timerActive)
{
    *timerActive = true;
    std::this_thread::sleep_for(std::chrono::seconds(10));
    *timerActive = false;
}