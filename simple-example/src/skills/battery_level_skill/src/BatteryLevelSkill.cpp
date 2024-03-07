/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "BatteryLevelSkill.h"

#include <QTimer>
#include <QDebug>
#include <QTime>
#include <iostream>
#include <QStateMachine>

BatteryLevelSkill::BatteryLevelSkill(std::string name ) :
        m_name(std::move(name))
{
    // m_stateMachine.setDataModel(&m_dataModel);
}


void BatteryLevelSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::spin(node);  
    rclcpp::shutdown();  
}


bool BatteryLevelSkill::start(int argc, char*argv[])
{

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }

    m_node = rclcpp::Node::make_shared(m_name + "Skill");

    
    RCLCPP_DEBUG_STREAM(m_node->get_logger(), "BatteryLevelSkill::start");
    std::cout << "BatteryLevelSkill::start";
    m_tickService = m_node->create_service<bt_interfaces::srv::TickCondition>(m_name + "Skill/tick",  
                                                                                std::bind(&BatteryLevelSkill::tick,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));

    m_stateMachine.start();
    m_threadSpin = std::make_shared<std::thread>(spin, m_node);
    m_subscription = m_node->create_subscription<sensor_msgs::msg::BatteryState>(
     "/battery_status", 10, std::bind(&BatteryLevelSkill::topic_callback, this, std::placeholders::_1));



    m_stateMachine.connectToEvent("tickReturn", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::string result = event.data().toMap()["result"].toString().toStdString();
        // RCLCPP_INFO(m_node->get_logger(), "BatteryLevelSkill::tickresponse----------------%s--------------------------------", result.c_str());

        if (result == "SUCCESS" )
        { 
            m_tickResult.store(Status::success);
        } else if (result == "FAILURE" )
        { 
            m_tickResult.store(Status::failure);
        }
    });

    return true;
}

void BatteryLevelSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickCondition::Request> request,
                                       std::shared_ptr<bt_interfaces::srv::TickCondition::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    // RCLCPP_INFO(m_node->get_logger(), "BatteryLevelSkill::tick");
    auto message = bt_interfaces::msg::ConditionResponse();
    m_tickResult.store(Status::undefined); //here we can put a struct
    m_stateMachine.submitEvent("tickCall");

    while(m_tickResult.load()== Status::undefined) 
    {
        // qInfo() <<  "active names" << m_stateMachine.activeStateNames();
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
    }
    switch(m_tickResult.load()) 
    {
        case Status::failure:
            response->status.status = message.SKILL_FAILURE;
            break;
        case Status::success:
            response->status.status = message.SKILL_SUCCESS;
            break;            
    }
    // RCLCPP_INFO(m_node->get_logger(), "BatteryLevelSkill::tickDone");

    response->is_ok = true;
}



void BatteryLevelSkill::topic_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    // RCLCPP_INFO(m_node->get_logger(), "BatteryLevelSkill::topic_callback");
    QVariantMap data;
    data.insert("result", msg->percentage);
    // foreach (auto key, data.keys())
    // {
    //     qInfo()<<"-- key:"<<key<<" value:"<<data.value(key);
    // }
    m_stateMachine.submitEvent("BatteryDriverCmpInterface.readLevel", data);

}