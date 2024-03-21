/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

# pragma once

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "BatteryLevelSkillSM.h"
#include "BatteryLevelSkillDataModel.h"
#include <bt_interfaces/msg/condition_response.hpp>
#include <bt_interfaces/srv/tick_condition.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <mutex>




enum class Status{
    undefined,
    success,
    failure
};


class BatteryLevelSkill
{
public:
    BatteryLevelSkill(std::string name );
    void topic_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    bool start(int argc, char * argv[]);
    static void spin(std::shared_ptr<rclcpp::Node> node);
    void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickCondition::Request> request,
          std::shared_ptr<bt_interfaces::srv::TickCondition::Response>      response);

private:
    std::shared_ptr<std::thread> m_threadSpin;
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Service<bt_interfaces::srv::TickCondition>::SharedPtr m_tickService;
    std::string m_name;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr m_subscription;
    std::atomic<Status> m_tickResult{Status::undefined};
    BatteryLevelSkillDataModel m_dataModel;

    BatteryLevelSkillSM m_stateMachine;
    std::mutex m_requestMutex;
};
