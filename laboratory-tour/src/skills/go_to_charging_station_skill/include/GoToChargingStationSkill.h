/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

# pragma once

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "GoToChargingStationSkillSM.h"
#include "GoToChargingStationDataModel.h"
#include <bt_interfaces/msg/condition_response.hpp>
#include <bt_interfaces/srv/tick_condition.hpp>
#include <mutex>

class GoToChargingStationSkill
{
public:
    GoToChargingStationSkill(std::string name );

    bool start(int argc, char * argv[]);
    static void spin(std::shared_ptr<rclcpp::Node> node);
    void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickCondition::Request> request,
          std::shared_ptr<bt_interfaces::srv::TickCondition::Response>      response);

private:
    std::shared_ptr<std::thread> m_threadSpin;
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Service<bt_interfaces::srv::TickCondition>::SharedPtr m_tickService;
    std::string m_name;
    GoToChargingStationDataModel m_dataModel;
    GoToChargingStationSkillSM m_stateMachine;
    std::mutex m_requestMutex;
};
