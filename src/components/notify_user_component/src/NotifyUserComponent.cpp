/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "NotifyUserComponent.h"

bool NotifyUserComponent::start(int argc, char*argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_alarmActive = std::make_shared<std::atomic<bool>>();
    *m_alarmActive = false;
    m_node = rclcpp::Node::make_shared("NotifyUserComponentNode");
    m_startAlarmService = m_node->create_service<notify_user_interfaces::srv::StartAlarm>("/NotifyUserComponent/StartAlarm",  
                                                                                std::bind(&NotifyUserComponent::StartAlarm,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_stopAlarmService = m_node->create_service<notify_user_interfaces::srv::StopAlarm>("/NotifyUserComponent/StopAlarm",  
                                                                                std::bind(&NotifyUserComponent::StopAlarm,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_notifyUserChargedService = m_node->create_service<notify_user_interfaces::srv::NotifyUserCharged>("/NotifyUserComponent/NotifyUserCharged",  
                                                                                std::bind(&NotifyUserComponent::NotifyUserCharged,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    RCLCPP_INFO(m_node->get_logger(), "NotifyUserComponent::start");
    return true;

}

bool NotifyUserComponent::close()
{
    rclcpp::shutdown();  
    return true;
}

void NotifyUserComponent::spin()
{
    rclcpp::spin(m_node);  
}

void NotifyUserComponent::StartAlarm([[maybe_unused]] const std::shared_ptr<notify_user_interfaces::srv::StartAlarm::Request> request,
             std::shared_ptr<notify_user_interfaces::srv::StartAlarm::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if ( *m_alarmActive ) {
        response->is_ok = true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------- alarm already active ------------------------------");
        return;
    }
    *m_alarmActive = true;
    m_threadAlarm = std::make_shared<std::thread>(Alarm,m_alarmActive);
    response->is_ok = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------- alarm started ------------------------------");
}


void NotifyUserComponent::StopAlarm([[maybe_unused]] const std::shared_ptr<notify_user_interfaces::srv::StopAlarm::Request> request,
             std::shared_ptr<notify_user_interfaces::srv::StopAlarm::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutex);
    *m_alarmActive = false;
    m_threadAlarm->join();
    response->is_ok = true;
}

void NotifyUserComponent::NotifyUserCharged([[maybe_unused]] const std::shared_ptr<notify_user_interfaces::srv::NotifyUserCharged::Request> request,
             std::shared_ptr<notify_user_interfaces::srv::NotifyUserCharged::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!*m_alarmActive) {
        *m_alarmActive = false;
        if(m_threadAlarm != nullptr && m_threadAlarm->joinable()){
            m_threadAlarm->join();
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I'm charged =======================================================================================================");
    response->is_ok = true;
}



void NotifyUserComponent::Alarm(std::shared_ptr<std::atomic<bool>> alarmActive) 
{
    while(*alarmActive) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "alarm");
        std::this_thread::sleep_for (std::chrono::milliseconds(500));
    }
    std::this_thread::sleep_for (std::chrono::milliseconds(500));

}