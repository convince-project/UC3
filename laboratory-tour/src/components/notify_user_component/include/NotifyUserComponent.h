/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <alarm_interfaces/srv/start_alarm.hpp>
#include <alarm_interfaces/srv/stop_alarm.hpp>
#include <notify_user_interfaces/srv/notify_user_charged.hpp>

class NotifyUserComponent
{
public:
    NotifyUserComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    static void Alarm(std::shared_ptr<std::atomic<bool>> alarmActive);
    void StopAlarm([[maybe_unused]] const std::shared_ptr<alarm_interfaces::srv::StopAlarm::Request> request,
                std::shared_ptr<alarm_interfaces::srv::StopAlarm::Response>      response);
    void StartAlarm( [[maybe_unused]] const std::shared_ptr<alarm_interfaces::srv::StartAlarm::Request> request,
                std::shared_ptr<alarm_interfaces::srv::StartAlarm::Response>      response);
    void NotifyUserCharged( [[maybe_unused]] const std::shared_ptr<notify_user_interfaces::srv::NotifyUserCharged::Request> request,
                std::shared_ptr<notify_user_interfaces::srv::NotifyUserCharged::Response>      response);

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<alarm_interfaces::srv::StopAlarm>::SharedPtr m_stopAlarmService;
    rclcpp::Service<alarm_interfaces::srv::StartAlarm>::SharedPtr m_startAlarmService;
    rclcpp::Service<notify_user_interfaces::srv::NotifyUserCharged>::SharedPtr m_notifyUserChargedService;
    std::mutex m_mutex;
    std::shared_ptr<std::thread> m_threadAlarm;
    std::shared_ptr<std::atomic<bool>> m_alarmActive;
};
