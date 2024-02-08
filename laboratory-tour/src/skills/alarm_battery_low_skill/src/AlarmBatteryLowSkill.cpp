/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "AlarmBatteryLowSkill.h"

#include <QTimer>
#include <QDebug>
#include <QTime>
#include <iostream>
#include <QStateMachine>

AlarmBatteryLowSkill::AlarmBatteryLowSkill(std::string name ) :
        m_name(std::move(name))
{
    //stateMachine.setDataModel(&dataModel);
}


void AlarmBatteryLowSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::spin(node);  
    rclcpp::shutdown();  
}


bool AlarmBatteryLowSkill::start(int argc, char*argv[])
{

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }

    m_node = rclcpp::Node::make_shared(m_name + "Skill");
    m_tickService = m_node->create_service<bt_interfaces::srv::TickAction>(m_name + "Skill/tick",  std::bind(&AlarmBatteryLowSkill::tick,
                                                                                                                 this,
                                                                                                                 std::placeholders::_1,
                                                                                                                 std::placeholders::_2));
    m_haltService = m_node->create_service<bt_interfaces::srv::HaltAction>(m_name + "Skill/halt",  std::bind(&AlarmBatteryLowSkill::halt,
                                                                                                                 this,
                                                                                                                 std::placeholders::_1,
                                                                                                                 std::placeholders::_2));
                                                                                                                 
    RCLCPP_DEBUG(m_node->get_logger(), "AlarmBatteryLowSkill::start");
    std::cout << "AlarmBatteryLowSkill::start";

    m_stateMachine.start();
    m_threadSpin = std::make_shared<std::thread>(spin, m_node);
    return true;
}



void AlarmBatteryLowSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
                                       std::shared_ptr<bt_interfaces::srv::TickAction::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_DEBUG(m_node->get_logger(), "AlarmBatteryLowSkill::tick");
    std::cout << "AlarmBatteryLowSkill::tick";
    for (const auto& state : m_stateMachine.activeStateNames()) {
            std::cout << state.toStdString() << std::endl;
            auto message = bt_interfaces::msg::ActionResponse();
            if (state == "alarm") {
                response->status.status = message.SKILL_SUCCESS;
            } else if (state == "idle") {
                response->status.status = message.SKILL_RUNNING; //here goes also the initialisation
                m_stateMachine.submitEvent("CMD_START");
            } else if (state == "ready") {
                response->status.status = message.SKILL_RUNNING;
                m_stateMachine.submitEvent("CMD_ALARM");
            }
    }
    response->is_ok = true;
}

void AlarmBatteryLowSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::HaltAction::Request> request,
               [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::HaltAction::Response> response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    bool halted = false;

    do {
        for (const auto& state : m_stateMachine.activeStateNames()) {
            RCLCPP_DEBUG_STREAM(m_node->get_logger(), state.toStdString());
            if (state == "idle") {
                halted = true;
            } else {
                m_stateMachine.submitEvent("CMD_HALT");
            }
        }
    } while (!halted);}
