/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "AlarmComponent.h"

bool AlarmComponent::start(int argc, char*argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    *m_alarmActive = false;
    m_node = rclcpp::Node::make_shared("AlarmComponentNode");
    m_startAlarmService = m_node->create_service<alarm_interfaces::srv::StartAlarm>("/AlarmComponent/StartAlarm",  
                                                                                std::bind(&AlarmComponent::StartAlarm,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_stopAlarmService = m_node->create_service<alarm_interfaces::srv::StopAlarm>("/AlarmComponent/StopAlarm",  
                                                                                std::bind(&AlarmComponent::StopAlarm,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    RCLCPP_DEBUG(m_node->get_logger(), "AlarmComponent::start");
    std::cout << "AlarmComponent::start";        
    return true;

}

bool AlarmComponent::close()
{
    rclcpp::shutdown();  
    return true;
}

void AlarmComponent::spin()
{
    rclcpp::spin(m_node);  
}

void AlarmComponent::StartAlarm([[maybe_unused]] const std::shared_ptr<alarm_interfaces::srv::StartAlarm::Request> request,
             std::shared_ptr<alarm_interfaces::srv::StartAlarm::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutex);
    *m_alarmActive = true;
    m_threadAlarm = std::make_shared<std::thread>(Alarm,m_alarmActive);
    response->is_ok = true;
}


void AlarmComponent::StopAlarm([[maybe_unused]] const std::shared_ptr<alarm_interfaces::srv::StopAlarm::Request> request,
             std::shared_ptr<alarm_interfaces::srv::StopAlarm::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutex);
    *m_alarmActive = false;
    m_threadAlarm->join();
    response->is_ok = true;
}


void AlarmComponent::Alarm(std::shared_ptr<std::atomic<bool>> alarmActive) 
{
    while(*alarmActive) {
        std::cout << "alarm";
        std::this_thread::sleep_for (std::chrono::milliseconds(500));
    }
    std::this_thread::sleep_for (std::chrono::milliseconds(500));
}