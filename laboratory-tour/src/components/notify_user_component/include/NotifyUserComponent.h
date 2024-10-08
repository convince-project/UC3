/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <notify_user_interfaces_dummy/srv/start_alarm.hpp>
#include <notify_user_interfaces_dummy/srv/stop_alarm.hpp>

class NotifyUserComponent
{
public:
    NotifyUserComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    static void Alarm(std::shared_ptr<std::atomic<bool>> alarmActive);
    void StopAlarm([[maybe_unused]] const std::shared_ptr<notify_user_interfaces_dummy::srv::StopAlarm::Request> request,
                std::shared_ptr<notify_user_interfaces_dummy::srv::StopAlarm::Response>      response);
    void StartAlarm( [[maybe_unused]] const std::shared_ptr<notify_user_interfaces_dummy::srv::StartAlarm::Request> request,
                std::shared_ptr<notify_user_interfaces_dummy::srv::StartAlarm::Response>      response);

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<notify_user_interfaces_dummy::srv::StopAlarm>::SharedPtr m_stopAlarmService;
    rclcpp::Service<notify_user_interfaces_dummy::srv::StartAlarm>::SharedPtr m_startAlarmService;
    std::mutex m_mutex;
    std::shared_ptr<std::thread> m_threadAlarm;
    std::shared_ptr<std::atomic<bool>> m_alarmActive;
    bool m_active;
};
