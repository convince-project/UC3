/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "AlarmBatteryLowSkill.h"
#include <future>
#include <QTimer>
#include <QDebug>
#include <QTime>
#include <iostream>
#include <QStateMachine>

AlarmBatteryLowSkill::AlarmBatteryLowSkill(std::string name ) :
        m_name(std::move(name))
{
    // m_stateMachine.setDataModel(&dataModel);
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
                                                                                                                 
    m_threadSpin = std::make_shared<std::thread>(spin, m_node);


    m_stateMachine.connectToEvent("AlarmCmpInterface.START_ALARM", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeStartAlarm = rclcpp::Node::make_shared(m_name + "SkillNodeStartAlarm");
        // RCLCPP_INFO(nodeStartAlarm->get_logger(), "start alarm");
        std::shared_ptr<rclcpp::Client<alarm_interfaces::srv::StartAlarm>> clientStartAlarm = nodeStartAlarm->create_client<alarm_interfaces::srv::StartAlarm>("/AlarmComponent/StartAlarm");

        auto request = std::make_shared<alarm_interfaces::srv::StartAlarm::Request>();
        bool wait_succeded{true};
        while (!clientStartAlarm->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service StartAlarm. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("AlarmCmpInterface.START_ALARM_FAILED");
            } 
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service StartAlarm not available, waiting again...");
        }

        if (wait_succeded) {
            auto result = clientStartAlarm->async_send_request(request);
            std::this_thread::sleep_for (std::chrono::milliseconds(100));
            if (rclcpp::spin_until_future_complete(nodeStartAlarm, result) ==
                rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( result.get()->is_ok ==true) {
                    m_stateMachine.submitEvent("AlarmCmpInterface.START_ALARM_SUCCEDED");
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AlarmCmpInterface.START_ALARM_SUCCEDED");
                } else {
                    m_stateMachine.submitEvent("AlarmCmpInterface.START_ALARM_FAILED");
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AlarmCmpInterface.START_ALARM_FAILED");
                }
            }       
        }
    });

    m_stateMachine.connectToEvent("AlarmCmpInterface.STOP_ALARM", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeStopAlarm = rclcpp::Node::make_shared(m_name + "SkillNodeStopAlarm");
        RCLCPP_INFO(nodeStopAlarm->get_logger(), "stop alarm");
        std::shared_ptr<rclcpp::Client<alarm_interfaces::srv::StopAlarm>> clientStopAlarm = nodeStopAlarm->create_client<alarm_interfaces::srv::StopAlarm>("/AlarmComponent/StopAlarm");

        auto request = std::make_shared<alarm_interfaces::srv::StopAlarm::Request>();
        bool wait_succeded{true};
        while (!clientStopAlarm->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service StopAlarm. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("AlarmCmpInterface.STOP_ALARM_FAILED");
            } 
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service StopAlarm not available, waiting again...");
        }

        if (wait_succeded) {
            auto result = clientStopAlarm->async_send_request(request);
            std::this_thread::sleep_for (std::chrono::milliseconds(100));
            if (rclcpp::spin_until_future_complete(nodeStopAlarm, result) ==
                rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( result.get()->is_ok ==true) {
                    m_stateMachine.submitEvent("AlarmCmpInterface.STOP_ALARM_SUCCEDED");
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STOP_SUCCEDED");
                } else {
                    m_stateMachine.submitEvent("AlarmCmpInterface.STOP_ALARM_FAILED");
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STOP_FAILED");
                }
            }       
        }
    });

    m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
        RCLCPP_INFO(m_node->get_logger(), "AlarmBatteryLowSkill::tickresponse");
        std::string result = event.data().toMap()["state"].toString().toStdString();
        if (result == "running" )
        { 
            m_tickResult.store(Status::running);
        } else if (result == "success" )
        { 
            m_tickResult.store(Status::success);
        } else if (result == "failure" )
        { 
            m_tickResult.store(Status::failure);
        }
    });


    m_stateMachine.connectToEvent("HALT_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
        RCLCPP_INFO(m_node->get_logger(), "AlarmBatteryLowSkill::haltresponse");
        m_haltResult.store(true);
    });

    m_stateMachine.start();
    return true;
}



void AlarmBatteryLowSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
                                       std::shared_ptr<bt_interfaces::srv::TickAction::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "AlarmBatteryLowSkill::tick");
    auto message = bt_interfaces::msg::ActionResponse();
    m_tickResult.store(Status::undefined); //here we can put a struct
    m_stateMachine.submitEvent("CMD_TICK");

    while(m_tickResult.load()== Status::undefined) 
    {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
        qInfo() <<  "active names" << m_stateMachine.activeStateNames();
    }
    switch(m_tickResult.load()) 
    {
        case Status::running:
            response->status.status = message.SKILL_RUNNING;
            break;
        case Status::failure:
            response->status.status = message.SKILL_FAILURE;
            break;
        case Status::success:
            response->status.status = message.SKILL_SUCCESS;
            break;            
    }
    RCLCPP_INFO(m_node->get_logger(), "AlarmBatteryLowSkill::tickDone");

    response->is_ok = true;
}

void AlarmBatteryLowSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::HaltAction::Request> request,
               [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::HaltAction::Response> response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "AlarmBatteryLowSkill::halt");
    m_haltResult.store(false); //here we can put a struct
    m_stateMachine.submitEvent("CMD_HALT");

    while(!m_haltResult.load()) 
    {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
        // qInfo() <<  "active names" << m_stateMachine.activeStateNames();
    }
    RCLCPP_INFO(m_node->get_logger(), "AlarmBatteryLowSkill::haltDone");

    response->is_ok = true;
}
