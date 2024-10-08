/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <timer_interfaces_dummy/srv/start_timer.hpp>
#include <timer_interfaces_dummy/srv/is_timer_active.hpp>

class TimerComponent
{
public:
    TimerComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    static void Timer(std::shared_ptr<std::atomic<bool>> timerActive);
    void StartTimer([[maybe_unused]] const std::shared_ptr<timer_interfaces_dummy::srv::StartTimer::Request> request,
                std::shared_ptr<timer_interfaces_dummy::srv::StartTimer::Response>      response);
    void IsTimerActive([[maybe_unused]] const std::shared_ptr<timer_interfaces_dummy::srv::IsTimerActive::Request> request,
                std::shared_ptr<timer_interfaces_dummy::srv::IsTimerActive::Response>      response);
    
private:
    rclcpp::Node::SharedPtr m_node;
    std::mutex m_mutex;
    rclcpp::Service<timer_interfaces_dummy::srv::StartTimer>::SharedPtr m_startTimerService;
    rclcpp::Service<timer_interfaces_dummy::srv::IsTimerActive>::SharedPtr m_isTimerActiveService;
    

    std::shared_ptr<std::thread> m_threadTimer;
    std::shared_ptr<std::atomic<bool>> m_timerActive;
};
