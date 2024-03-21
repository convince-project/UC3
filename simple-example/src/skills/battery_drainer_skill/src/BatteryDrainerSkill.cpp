/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "BatteryDrainerSkill.h"
#include <drain_interfaces/srv/drain.hpp>
#include <future>
#include <QTimer>
#include <QDebug>
#include <QTime>
#include <iostream>
#include <QStateMachine>

BatteryDrainerSkill::BatteryDrainerSkill(std::string name ) :
        m_name(std::move(name))
{
    m_stateMachine.setDataModel(&m_dataModel);
}


void BatteryDrainerSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::spin(node);  
    rclcpp::shutdown();  
}


bool BatteryDrainerSkill::start(int argc, char*argv[])
{

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
                                                                                                                 
    m_threadSpin = std::make_shared<std::thread>(spin, m_node);

    m_stateMachine.connectToEvent("BatteryDriverCmp.drainCall", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeBatteryDrainer = rclcpp::Node::make_shared(m_name + "SkillNodeBatteryDrainer");
        // RCLCPP_INFO(nodeBatteryDrainer->get_logger(), "BatteryDriverCmp.drainCall");

        std::shared_ptr<rclcpp::Client<drain_interfaces::srv::Drain>> clientBatteryDrainer = nodeBatteryDrainer->create_client<drain_interfaces::srv::Drain>("/BatteryDrainerComponent/Drain");

        // control if the service is active or not
        auto request = std::make_shared<drain_interfaces::srv::Drain::Request>();
        bool wait_succeded{true};
        while (!clientBatteryDrainer->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service BatteryDrainer. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("BatteryDriverCmp.drainReturn");
            } 
        }

        if (wait_succeded) {
            // send the request
            auto result = clientBatteryDrainer->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeBatteryDrainer, result);
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( result.get()->is_ok ==true) {
                    m_stateMachine.submitEvent("BatteryDriverCmp.drainReturn");
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BatteryDriverCmp.drainReturn");
                } else {
                    m_stateMachine.submitEvent("BatteryDriverCmp.drainReturn");
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BatteryDriverCmp.drainReturn");
                }
            }
        }
    });

    m_stateMachine.connectToEvent("tickReturn", [this]([[maybe_unused]]const QScxmlEvent & event){
        RCLCPP_INFO(m_node->get_logger(), "BatteryDrainerSkill::tickReturn %s", event.data().toMap()["result"].toString().toStdString().c_str());
        std::string result = event.data().toMap()["result"].toString().toStdString();
        if (result == "RUNNING" )
        { 
            m_tickResult.store(Status::running);
        } else if (result == "SUCCESS" )
        { 
            m_tickResult.store(Status::success);
        } else if (result == "FAILURE" )
        { 
            m_tickResult.store(Status::failure);
        }
    });


    m_stateMachine.connectToEvent("haltReturn", [this]([[maybe_unused]]const QScxmlEvent & event){
        RCLCPP_INFO(m_node->get_logger(), "BatteryDrainerSkill::haltresponse");
        m_haltResult.store(true);
    });

    m_stateMachine.start();
    return true;
}



void BatteryDrainerSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
                                       std::shared_ptr<bt_interfaces::srv::TickAction::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "BatteryDrainerSkill::tick");
    auto message = bt_interfaces::msg::ActionResponse();
    m_tickResult.store(Status::undefined); //here we can put a struct
    m_stateMachine.submitEvent("tickCall");

    while(m_tickResult.load()== Status::undefined) 
    {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
        // qInfo() <<  "active names" << m_stateMachine.activeStateNames();
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
    RCLCPP_INFO(m_node->get_logger(), "BatteryDrainerSkill::tickDone");

    response->is_ok = true;
}

void BatteryDrainerSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::HaltAction::Request> request,
               [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::HaltAction::Response> response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "BatteryDrainerSkill::halt");
    m_haltResult.store(false); //here we can put a struct
    m_stateMachine.submitEvent("haltCall");

    while(!m_haltResult.load()) 
    {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
        // qInfo() <<  "active names" << m_stateMachine.activeStateNames();
    }
    RCLCPP_INFO(m_node->get_logger(), "BatteryDrainerSkill::haltDone");

    response->is_ok = true;
}


// [INFO] [1709805752.863414800] [BatteryDrainerSkill]: BatteryDrainerSkill::tick
// [INFO] [1709805752.867011108] [BatteryDrainerSkillNodeBatteryDrainer]: BatteryDriverCmp.drainCall
// [INFO] [1709805752.871942992] [BatteryDrainerSkill]: BatteryDrainerSkill::tickReturn
