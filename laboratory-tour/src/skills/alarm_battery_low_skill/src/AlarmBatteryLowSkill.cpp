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
    m_clientStartAlarm = m_node->create_client<alarm_interfaces::srv::StartAlarm>(m_name + "Component/StartAlarm");

    m_stateMachine.connectToEvent("SEND_ALARM", [this]([[maybe_unused]]const QScxmlEvent & event){
    auto request = std::make_shared<alarm_interfaces::srv::StartAlarm::Request>();
    bool wait_succeded{false};
    while (!m_clientStartAlarm->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service StartAlarm. Exiting.");
            m_stateMachine.submitEvent("START_FAILED");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service StartAlarm not available, waiting again...");
    }
    if (wait_succeded) {
        auto result = m_clientStartAlarm->async_send_request(request);
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
        if (rclcpp::spin_until_future_complete(m_node, result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            if( result.get()->is_ok ==true) {
                m_stateMachine.submitEvent("START_SUCCEDED");
            } else {
                m_stateMachine.submitEvent("START_FAILED");
            }
        }
    }
    });
    return true;
}



void AlarmBatteryLowSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
                                       std::shared_ptr<bt_interfaces::srv::TickAction::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_DEBUG(m_node->get_logger(), "AlarmBatteryLowSkill::tick");
    auto message = bt_interfaces::msg::ActionResponse();
    m_stateMachine.submitEvent("CMD_TICK");
    std::shared_ptr<std::promise<std::string>> futureResult;
    //to check if this doesn't add other callbacks
    m_stateMachine.connectToEvent("TICK_RESPONSE", [futureResult](const QScxmlEvent & event){
        futureResult->set_value(event.data().toMap()["state"].toString().toStdString());
        std::cout << "received!" << std::endl;
    }, Qt::SingleShotConnection);
    std::future<std::string> future = futureResult->get_future();
    future.wait();
    std::string status = future.get();
    if (status == "running") {
        response->status.status = message.SKILL_RUNNING;
    } else if (status == "failure") {
        response->status.status = message.SKILL_FAILURE;
    } else if (status == "success") {
        response->status.status = message.SKILL_SUCCESS;
    }
    response->is_ok = true;
}

void AlarmBatteryLowSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::HaltAction::Request> request,
               [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::HaltAction::Response> response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    m_stateMachine.submitEvent("CMD_HALT");
    std::shared_ptr<std::promise<bool>> futureResult;
    m_stateMachine.connectToEvent("HALT_RESPONSE", [futureResult]([[maybe_unused]]const QScxmlEvent & event){
        futureResult->set_value(true);
        std::cout << "received!" << std::endl;
    },Qt::SingleShotConnection);
    std::future<bool> future = futureResult->get_future();
    future.wait();
    response->is_ok = true;
}
