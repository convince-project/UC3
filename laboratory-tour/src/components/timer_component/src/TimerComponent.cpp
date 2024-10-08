/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "TimerComponent.h"

bool TimerComponent::start(int argc, char*argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_timerActive = std::make_shared<std::atomic<bool>>();
    *m_timerActive = false;
    m_node = rclcpp::Node::make_shared("TimerComponentNode");
    m_startTimerService = m_node->create_service<timer_interfaces_dummy::srv::StartTimer>("/TimerComponent/StartTimer",  
                                                                                std::bind(&TimerComponent::StartTimer,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_isTimerActiveService = m_node->create_service<timer_interfaces_dummy::srv::IsTimerActive>("/TimerComponent/IsTimerActive",  
                                                                                std::bind(&TimerComponent::IsTimerActive,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    
    RCLCPP_INFO(m_node->get_logger(), "TimerComponent::start");
    return true;

}

bool TimerComponent::close()
{
    rclcpp::shutdown();  
    return true;
}

void TimerComponent::spin()
{
    rclcpp::spin(m_node);  
}

void TimerComponent::IsTimerActive([[maybe_unused]] const std::shared_ptr<timer_interfaces_dummy::srv::IsTimerActive::Request> request,
                std::shared_ptr<timer_interfaces_dummy::srv::IsTimerActive::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutex);
    response->is_active = *m_timerActive;
    response->is_ok = true;
}

void TimerComponent::StartTimer([[maybe_unused]] const std::shared_ptr<timer_interfaces_dummy::srv::StartTimer::Request> request,
                std::shared_ptr<timer_interfaces_dummy::srv::StartTimer::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (*m_timerActive) {
        response->is_ok = false;
        // response->error_msg = "Timer already active";
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



void TimerComponent::Timer(std::shared_ptr<std::atomic<bool>> timerActive)
{
    *timerActive = true;
    std::this_thread::sleep_for(std::chrono::seconds(30));
    *timerActive = false;
}
