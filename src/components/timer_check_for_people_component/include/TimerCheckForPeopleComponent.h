/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <timer_check_for_people_interfaces/srv/start_timer.hpp>
#include <timer_check_for_people_interfaces/srv/is_timer_active.hpp>

class TimerCheckForPeopleComponent
{
public:
    TimerCheckForPeopleComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    static void Timer(std::shared_ptr<std::atomic<bool>> timerActive);
    void StartTimer([[maybe_unused]] const std::shared_ptr<timer_check_for_people_interfaces::srv::StartTimer::Request> request,
                std::shared_ptr<timer_check_for_people_interfaces::srv::StartTimer::Response>      response);
    void IsTimerActive([[maybe_unused]] const std::shared_ptr<timer_check_for_people_interfaces::srv::IsTimerActive::Request> request,
                std::shared_ptr<timer_check_for_people_interfaces::srv::IsTimerActive::Response>      response);

private:
    rclcpp::Node::SharedPtr m_node;
    std::mutex m_mutex;
    rclcpp::Service<timer_check_for_people_interfaces::srv::StartTimer>::SharedPtr m_startTimerService;
    rclcpp::Service<timer_check_for_people_interfaces::srv::IsTimerActive>::SharedPtr m_isTimerActiveService;
    std::shared_ptr<std::thread> m_threadTimer;
    std::shared_ptr<std::atomic<bool>> m_timerActive;
};
