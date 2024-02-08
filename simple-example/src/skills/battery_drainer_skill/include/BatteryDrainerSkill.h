/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

# pragma once

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "BatteryDrainerSkillSM.h"
#include "BatteryDrainerDataModel.h"
#include <bt_interfaces/msg/action_response.hpp>
#include <bt_interfaces/srv/tick_action.hpp>
#include <bt_interfaces/srv/halt_action.hpp>
#include <mutex>

class BatteryDrainerSkill
{
public:
    BatteryDrainerSkill(std::string name );

    bool start(int argc, char * argv[]);
    static void spin(std::shared_ptr<rclcpp::Node> node);
    void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
               std::shared_ptr<bt_interfaces::srv::TickAction::Response> response);
    void halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::HaltAction::Request> request,
               [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::HaltAction::Response> response);

private:
    std::shared_ptr<std::thread> m_threadSpin;
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Service<bt_interfaces::srv::TickAction>::SharedPtr m_tickService;
    rclcpp::Service<bt_interfaces::srv::HaltAction>::SharedPtr m_haltService;
    std::string m_name;
    BatteryDrainerDataModel m_dataModel;
    BatteryDrainerSkillSM m_stateMachine;
    std::mutex m_requestMutex;
};
