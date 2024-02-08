/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "BatteryDrainerSkill.h"

#include <QTimer>
#include <QDebug>
#include <QTime>
#include <iostream>
#include <QStateMachine>

BatteryDrainerSkill::BatteryDrainerSkill(std::string name ) :
        m_name(std::move(name))
{
    m_dataModel.set_name(m_name);
    m_stateMachine.setDataModel(&m_dataModel);
}


void BatteryDrainerSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::spin(node);  
    rclcpp::shutdown();  
}


bool BatteryDrainerSkill::start(int argc, char*argv[])
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }

    m_node = rclcpp::Node::make_shared(m_name + "Skill");
    m_tickService = m_node->create_service<bt_interfaces::srv::TickAction>(m_name + "Skill/tick",  std::bind(&BatteryDrainerSkill::tick,
                                                                                                                 this,
                                                                                                                 std::placeholders::_1,
                                                                                                                 std::placeholders::_2));
    m_haltService = m_node->create_service<bt_interfaces::srv::HaltAction>(m_name + "Skill/halt",  std::bind(&BatteryDrainerSkill::halt,
                                                                                                                 this,
                                                                                                                 std::placeholders::_1,
                                                                                                                 std::placeholders::_2));
                                                                                                                 

    m_stateMachine.start();
    m_threadSpin = std::make_shared<std::thread>(spin, m_node);
    return true;
}


void BatteryDrainerSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
                                       std::shared_ptr<bt_interfaces::srv::TickAction::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_DEBUG_STREAM(m_node->get_logger(), "BatteryDrainerSkill::tick" << m_stateMachine.activeStateNames().size());

    for (const auto& state : m_stateMachine.activeStateNames()) {
        RCLCPP_DEBUG_STREAM(m_node->get_logger(), state.toStdString());
        auto message = bt_interfaces::msg::ActionResponse();
        if (state == "drain") {
            response->status.status = message.SKILL_RUNNING;
            m_stateMachine.submitEvent("CMD_SUCCESS");
            break;
        } else if (state == "idle") {
            response->status.status = message.SKILL_RUNNING; //here goes also the initialisation
            m_stateMachine.submitEvent("CMD_START");
            break;
        } else if (state == "active") {
            response->status.status = message.SKILL_RUNNING;
            m_stateMachine.submitEvent("CMD_DRAIN");
            break;
        } else if (state == "success") {
            response->status.status = message.SKILL_SUCCESS;
            m_stateMachine.submitEvent("CMD_OK");
            break;
        }
    }
    response->is_ok = true;
}

void BatteryDrainerSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::HaltAction::Request> request,
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
    } while (!halted);
}
