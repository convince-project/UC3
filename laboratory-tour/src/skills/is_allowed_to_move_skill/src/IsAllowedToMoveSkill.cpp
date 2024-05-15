/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "IsAllowedToMoveSkill.h"
#include <future>
#include <QTimer>
#include <QDebug>
#include <QTime>
#include <iostream>
#include <QStateMachine>

#include <type_traits>

template<typename T>
T convert(const std::string& str) {
    if constexpr (std::is_same_v<T, int>) {
        return std::stoi(str);
    } else if constexpr (std::is_same_v<T, double>) {
        return std::stod(str);
    } else if constexpr (std::is_same_v<T, float>) {
        return std::stof(str);
    } 
    else if constexpr (std::is_same_v<T, bool>) { 
        if (str == "true" || str == "1") { 
            return true; 
        } else if (str == "false" || str == "0") { 
            return false; 
        } else { 
            throw std::invalid_argument("Invalid boolean value"); 
        } 
    } 
    else if constexpr (std::is_same_v<T, std::string>) {
        return str;
    }
    else {
        // Handle unsupported types
        throw std::invalid_argument("Unsupported type conversion");
    }
}

IsAllowedToMoveSkill::IsAllowedToMoveSkill(std::string name ) :
		m_name(std::move(name))
{
}

void IsAllowedToMoveSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool IsAllowedToMoveSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "IsAllowedToMoveSkill::start");
	std::cout << "IsAllowedToMoveSkill::start";

	m_tickService = m_node->create_service<bt_interfaces::srv::TickCondition>(m_name + "Skill/tick",
                                                                           	std::bind(&IsAllowedToMoveSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));

    m_stateMachine.connectToEvent("WaitWakeUpComponent.IsAllowedToMove.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeIsAllowedToMove = rclcpp::Node::make_shared(m_name + "SkillNodeIsAllowedToMove");
        std::shared_ptr<rclcpp::Client<allowed_to_move_interfaces::srv::IsAllowedToMove>> clientIsAllowedToMove = nodeIsAllowedToMove->create_client<allowed_to_move_interfaces::srv::IsAllowedToMove>("/WaitWakeUpComponent/IsAllowedToMove");
        auto request = std::make_shared<allowed_to_move_interfaces::srv::IsAllowedToMove::Request>();
        bool wait_succeded{true};
        int retries = 0;
        while (!clientIsAllowedToMove->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'IsAllowedToMove'. Exiting.");
                wait_succeded = false;
                break;
            } 
            retries++;
            if(retries == SERVICE_TIMEOUT) {
               RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'UpdatePoi'.");
               wait_succeded = false;
               break;
            }
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientIsAllowedToMove->async_send_request(request);
            const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
            auto futureResult = rclcpp::spin_until_future_complete(nodeIsAllowedToMove, result, timeout_duration);
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
               auto response = result.get();
               if( response->is_ok ==true) {
                   QVariantMap data;
                   data.insert("result", "SUCCESS");
                    data.insert("is_allowed_to_move", response->is_allowed_to_move);
                   m_stateMachine.submitEvent("WaitWakeUpComponent.IsAllowedToMove.Return", data);
                   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "WaitWakeUpComponent.IsAllowedToMove.Return");
                   return;
               }
           }
           else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
               RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'UpdatePoi'.");
           }
        }
       QVariantMap data;
       data.insert("result", "FAILURE");
       m_stateMachine.submitEvent("WaitWakeUpComponent.IsAllowedToMove.Return", data);
       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "WaitWakeUpComponent.IsAllowedToMove.Return");
    });

	m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
		RCLCPP_INFO(m_node->get_logger(), "IsAllowedToMoveSkill::tickReturn %s", event.data().toMap()["result"].toString().toStdString().c_str());
		std::string result = event.data().toMap()["result"].toString().toStdString();
		if (result == "SUCCESS" )
		{
			m_tickResult.store(Status::success);
		}
		else if (result == "FAILURE" )
		{ 
			m_tickResult.store(Status::failure);
		}
	});

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void IsAllowedToMoveSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickCondition::Request> request,
                                std::shared_ptr<bt_interfaces::srv::TickCondition::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "IsAllowedToMoveSkill::tick");
    auto message = bt_interfaces::msg::ConditionResponse();
    m_tickResult.store(Status::undefined); //here we can put a struct
    m_stateMachine.submitEvent("CMD_TICK");
   
    while(m_tickResult.load()== Status::undefined) 
    {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
        // qInfo() <<  "active names" << m_stateMachine.activeStateNames();
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
    RCLCPP_INFO(m_node->get_logger(), "IsAllowedToMoveSkill::tickDone");
   
    response->is_ok = true;
}

