/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "StopAndTurnBackSkill.h"
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
    else if constexpr (std::is_same_v<T, std::string>) {
        return str;
    }
    else {
        // Handle unsupported types
        throw std::invalid_argument("Unsupported type conversion");
    }
}

StopAndTurnBackSkill::StopAndTurnBackSkill(std::string name ) :
		m_name(std::move(name))
{
}

void StopAndTurnBackSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool StopAndTurnBackSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "StopAndTurnBackSkill::start");
	std::cout << "StopAndTurnBackSkill::start";

	m_tickService = m_node->create_service<bt_interfaces::srv::TickAction>(m_name + "Skill/tick",
                                                                           	std::bind(&StopAndTurnBackSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));

	m_haltService = m_node->create_service<bt_interfaces::srv::HaltAction>(m_name + "Skill/halt",
                                                                            	std::bind(&StopAndTurnBackSkill::halt,
                                                                            	this,
                                                                            	std::placeholders::_1,
                                                                            	std::placeholders::_2));

    m_stateMachine.connectToEvent("NavigationComponent.TurnBack.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeTurnBack = rclcpp::Node::make_shared(m_name + "SkillNodeTurnBack");
        std::shared_ptr<rclcpp::Client<navigation_interfaces::srv::TurnBack>> clientTurnBack = nodeTurnBack->create_client<navigation_interfaces::srv::TurnBack>("/NavigationComponent/TurnBack");
        auto request = std::make_shared<navigation_interfaces::srv::TurnBack::Request>();
        bool wait_succeded{true};
        while (!clientTurnBack->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'TurnBack'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("NavigationComponent.TurnBack.Return");
            } 
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientTurnBack->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeTurnBack, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    m_stateMachine.submitEvent("NavigationComponent.TurnBack.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NavigationComponent.TurnBack.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("NavigationComponent.TurnBack.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NavigationComponent.TurnBack.Return");
                }
            }
        }
    });

    m_stateMachine.connectToEvent("NavigationComponent.GetNavigationStatus.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeGetNavigationStatus = rclcpp::Node::make_shared(m_name + "SkillNodeGetNavigationStatus");
        std::shared_ptr<rclcpp::Client<navigation_interfaces::srv::GetNavigationStatus>> clientGetNavigationStatus = nodeGetNavigationStatus->create_client<navigation_interfaces::srv::GetNavigationStatus>("/NavigationComponent/GetNavigationStatus");
        auto request = std::make_shared<navigation_interfaces::srv::GetNavigationStatus::Request>();
        bool wait_succeded{true};
        while (!clientGetNavigationStatus->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GetNavigationStatus'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("NavigationComponent.GetNavigationStatus.Return");
            } 
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientGetNavigationStatus->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeGetNavigationStatus, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    data.insert("status", response->status.status);
                    m_stateMachine.submitEvent("NavigationComponent.GetNavigationStatus.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NavigationComponent.GetNavigationStatus.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("NavigationComponent.GetNavigationStatus.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NavigationComponent.GetNavigationStatus.Return");
                }
            }
        }
    });

    m_stateMachine.connectToEvent("BlackboardComponent.SetInt.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeSetInt = rclcpp::Node::make_shared(m_name + "SkillNodeSetInt");
        std::shared_ptr<rclcpp::Client<blackboard_interfaces::srv::SetIntBlackboard>> clientSetInt = nodeSetInt->create_client<blackboard_interfaces::srv::SetIntBlackboard>("/BlackboardComponent/SetInt");
        auto request = std::make_shared<blackboard_interfaces::srv::SetIntBlackboard::Request>();
        auto eventParams = event.data().toMap();
        request->field_name = convert<decltype(request->field_name)>(eventParams["field_name"].toString().toStdString());
        request->value = convert<decltype(request->value)>(eventParams["value"].toString().toStdString());
        bool wait_succeded{true};
        while (!clientSetInt->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'SetInt'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("BlackboardComponent.SetInt.Return");
            } 
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientSetInt->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeSetInt, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    m_stateMachine.submitEvent("BlackboardComponent.SetInt.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BlackboardComponent.SetInt.Return.Succs");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("BlackboardComponent.SetInt.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BlackboardComponent.SetInt.Return");
                }
            }
        }
    });

	m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
		RCLCPP_INFO(m_node->get_logger(), "StopAndTurnBackSkill::tickReturn %s", event.data().toMap()["result"].toString().toStdString().c_str());
		std::string result = event.data().toMap()["result"].toString().toStdString();
		if (result == "SUCCESS" )
		{
			m_tickResult.store(Status::success);
		}
		else if (result == "RUNNING" )
		{
			m_tickResult.store(Status::running);
		}
		else if (result == "FAILURE" )
		{ 
			m_tickResult.store(Status::failure);
		}
	});

	m_stateMachine.connectToEvent("HALT_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
		RCLCPP_INFO(m_node->get_logger(), "StopAndTurnBackSkill::haltresponse");
		m_haltResult.store(true);
	});

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void StopAndTurnBackSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
                                std::shared_ptr<bt_interfaces::srv::TickAction::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "StopAndTurnBackSkill::tick");
    auto message = bt_interfaces::msg::ActionResponse();
    m_tickResult.store(Status::undefined); //here we can put a struct
    m_stateMachine.submitEvent("CMD_TICK");
   
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
    RCLCPP_INFO(m_node->get_logger(), "StopAndTurnBackSkill::tickDone");
   
    response->is_ok = true;
}

void StopAndTurnBackSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::HaltAction::Request> request,
    [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::HaltAction::Response> response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "StopAndTurnBackSkill::halt");
    m_haltResult.store(false); //here we can put a struct
    m_stateMachine.submitEvent("CMD_HALT");
   
    while(!m_haltResult.load()) 
    {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
        // qInfo() <<  "active names" << m_stateMachine.activeStateNames();
    }
    RCLCPP_INFO(m_node->get_logger(), "StopAndTurnBackSkill::haltDone");
   
    response->is_ok = true;
}
