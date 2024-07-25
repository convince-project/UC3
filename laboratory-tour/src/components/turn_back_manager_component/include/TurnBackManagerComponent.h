/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <people_detector_filter_interfaces/msg/filter_status.hpp>
#include <turn_back_manager_interfaces/srv/reset_counters.hpp>
#include <turn_back_manager_interfaces/srv/set_max_turn_backs.hpp>
#include <turn_back_manager_interfaces/srv/get_max_turn_backs.hpp>
#include <turn_back_manager_interfaces/srv/set_max_consecutive_falses.hpp>
#include <turn_back_manager_interfaces/srv/get_max_consecutive_falses.hpp>
#include <turn_back_manager_interfaces/srv/increase_turn_backs_counter.hpp>
#include <turn_back_manager_interfaces/srv/get_turn_backs_counter.hpp>
#include <turn_back_manager_interfaces/srv/is_allowed_to_continue.hpp>
#include <turn_back_manager_interfaces/srv/is_allowed_to_turn_back.hpp>


class TurnBackManagerComponent 
{
public:
    TurnBackManagerComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    void topic_callback_people_detector(const std_msgs::msg::Bool::SharedPtr msg);
    void SetMaxTurnBacks( const std::shared_ptr<turn_back_manager_interfaces::srv::SetMaxTurnBacks::Request> request,
                std::shared_ptr<turn_back_manager_interfaces::srv::SetMaxTurnBacks::Response>      response);
    void GetMaxTurnBacks([[maybe_unused]] const std::shared_ptr<turn_back_manager_interfaces::srv::GetMaxTurnBacks::Request> request,
                std::shared_ptr<turn_back_manager_interfaces::srv::GetMaxTurnBacks::Response>      response);
    void SetMaxConsecutiveFalses( const std::shared_ptr<turn_back_manager_interfaces::srv::SetMaxConsecutiveFalses::Request> request,
                std::shared_ptr<turn_back_manager_interfaces::srv::SetMaxConsecutiveFalses::Response>      response);
    void GetMaxConsecutiveFalses([[maybe_unused]] const std::shared_ptr<turn_back_manager_interfaces::srv::GetMaxConsecutiveFalses::Request> request,
                std::shared_ptr<turn_back_manager_interfaces::srv::GetMaxConsecutiveFalses::Response>      response);
    void ResetCounters([[maybe_unused]] const std::shared_ptr<turn_back_manager_interfaces::srv::ResetCounters::Request> request,
                std::shared_ptr<turn_back_manager_interfaces::srv::ResetCounters::Response>      response);
    void IncreaseTurnBacksCounter([[maybe_unused]] const std::shared_ptr<turn_back_manager_interfaces::srv::IncreaseTurnBacksCounter::Request> request,
                std::shared_ptr<turn_back_manager_interfaces::srv::IncreaseTurnBacksCounter::Response>      response);
    void GetTurnBacksCounter([[maybe_unused]] const std::shared_ptr<turn_back_manager_interfaces::srv::GetTurnBacksCounter::Request> request,
                std::shared_ptr<turn_back_manager_interfaces::srv::GetTurnBacksCounter::Response>      response);
    void IsAllowedToContinue([[maybe_unused]] const std::shared_ptr<turn_back_manager_interfaces::srv::IsAllowedToContinue::Request> request,
                std::shared_ptr<turn_back_manager_interfaces::srv::IsAllowedToContinue::Response>      response);
    void IsAllowedToTurnBack([[maybe_unused]] const std::shared_ptr<turn_back_manager_interfaces::srv::IsAllowedToTurnBack::Request> request,
                std::shared_ptr<turn_back_manager_interfaces::srv::IsAllowedToTurnBack::Response>      response);

private:

    void counterTask();

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<turn_back_manager_interfaces::srv::ResetCounters>::SharedPtr m_resetCountersService;
    rclcpp::Service<turn_back_manager_interfaces::srv::SetMaxTurnBacks>::SharedPtr m_setMaxTurnBacksService;
    rclcpp::Service<turn_back_manager_interfaces::srv::GetMaxTurnBacks>::SharedPtr m_getMaxTurnBacksService;
    rclcpp::Service<turn_back_manager_interfaces::srv::SetMaxConsecutiveFalses>::SharedPtr m_setMaxConsecutiveFalsesService;
    rclcpp::Service<turn_back_manager_interfaces::srv::GetMaxConsecutiveFalses>::SharedPtr m_getMaxConsecutiveFalsesService;
    rclcpp::Service<turn_back_manager_interfaces::srv::IncreaseTurnBacksCounter>::SharedPtr m_increaseTurnBacksCounterService;
    rclcpp::Service<turn_back_manager_interfaces::srv::GetTurnBacksCounter>::SharedPtr m_getTurnBacksCounterService;
    rclcpp::Service<turn_back_manager_interfaces::srv::IsAllowedToContinue>::SharedPtr m_isAllowedToContinueService;
    rclcpp::Service<turn_back_manager_interfaces::srv::IsAllowedToTurnBack>::SharedPtr m_isAllowedToTurnBackService;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscriptionPeopleDetector;
    
    std::mutex m_mutex;
    std::mutex m_taskMutex;
    int32_t m_maxNumberTurnBacks{1};            //Max number of turn backs between two PoIs
    int32_t m_countNumberTurnBacks{0};          //Count number of turn backs between two PoIs
    int32_t m_maxNumberConsecutiveFalse{30};    //Max number of consecutive falses after turning back
    int32_t m_countNumberConsecutiveFalse{0};   //Count number of consecutive falses after turning back
    bool m_counterTask{false};
    std::thread m_threadCounter;
    bool m_lastDetection;
};
