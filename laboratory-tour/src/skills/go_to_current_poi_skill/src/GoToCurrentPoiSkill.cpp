/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "GoToCurrentPoiSkill.h"
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

GoToCurrentPoiSkill::GoToCurrentPoiSkill(std::string name ) :
		m_name(std::move(name))
{
}

void GoToCurrentPoiSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool GoToCurrentPoiSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "GoToCurrentPoiSkill::start");
	std::cout << "GoToCurrentPoiSkill::start";

	m_tickService = m_node->create_service<bt_interfaces::srv::TickAction>(m_name + "Skill/tick",
                                                                           	std::bind(&GoToCurrentPoiSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));

    m_stateMachine.connectToEvent("SchedulerComponent.GetCurrentPoi.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeGetCurrentPoi = rclcpp::Node::make_shared(m_name + "SkillNodeGetCurrentPoi");
        std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::GetCurrentPoi>> clientGetCurrentPoi = nodeGetCurrentPoi->create_client<scheduler_interfaces::srv::GetCurrentPoi>("/SchedulerComponent/GetCurrentPoi");
        auto request = std::make_shared<scheduler_interfaces::srv::GetCurrentPoi::Request>();
        bool wait_succeded{true};
        while (!clientGetCurrentPoi->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GetCurrentPoi'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("SchedulerComponent.GetCurrentPoi.Return");
            } 
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientGetCurrentPoi->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeGetCurrentPoi, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    data.insert("poi_name", response->poi_name.c_str());
                    std::cout << "poi_name: " << response->poi_name.c_str() << std::endl;
                    m_stateMachine.submitEvent("SchedulerComponent.GetCurrentPoi.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.GetCurrentPoi.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("SchedulerComponent.GetCurrentPoi.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.GetCurrentPoi.Return");
                }
            }
        }
    });

    m_stateMachine.connectToEvent("NavigationComponent.GoToPoiByName.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NavigationComponent.GoToPoiByName.Call");
        std::shared_ptr<rclcpp::Node> nodeGoToPoiByName = rclcpp::Node::make_shared(m_name + "SkillNodeGoToPoiByName");
        std::shared_ptr<rclcpp::Client<navigation_interfaces::srv::GoToPoiByName>> clientGoToPoiByName = nodeGoToPoiByName->create_client<navigation_interfaces::srv::GoToPoiByName>("/NavigationComponent/GoToPoiByName");
        auto request = std::make_shared<navigation_interfaces::srv::GoToPoiByName::Request>();
        auto eventParams = event.data().toMap();
        request->poi_name = convert<decltype(request->poi_name)>(eventParams["poi_name"].toString().toStdString());
        std::cout << "poi_name: " << request->poi_name.c_str() << std::endl;
        bool wait_succeded{true};
        while (!clientGoToPoiByName->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GoToPoiByName'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("NavigationComponent.GoToPoiByName.Return");
            } 
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientGoToPoiByName->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeGoToPoiByName, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    m_stateMachine.submitEvent("NavigationComponent.GoToPoiByName.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NavigationComponent.GoToPoiByName.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("NavigationComponent.GoToPoiByName.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NavigationComponent.GoToPoiByName.Return");
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

    m_stateMachine.connectToEvent("NavigationComponent.CheckNearToPoi.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeCheckNearToPoi = rclcpp::Node::make_shared(m_name + "SkillNodeCheckNearToPoi");
        std::shared_ptr<rclcpp::Client<navigation_interfaces::srv::CheckNearToPoi>> clientCheckNearToPoi = nodeCheckNearToPoi->create_client<navigation_interfaces::srv::CheckNearToPoi>("/NavigationComponent/CheckNearToPoi");
        auto request = std::make_shared<navigation_interfaces::srv::CheckNearToPoi::Request>();
        auto eventParams = event.data().toMap();
        request->distance = convert<decltype(request->distance)>(eventParams["distance"].toString().toStdString());
        request->poi_name = convert<decltype(request->poi_name)>(eventParams["poi_name"].toString().toStdString());
        bool wait_succeded{true};
        while (!clientCheckNearToPoi->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'CheckNearToPoi'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("NavigationComponent.CheckNearToPoi.Return");
            } 
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientCheckNearToPoi->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeCheckNearToPoi, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    data.insert("is_near", response->is_near);
                    m_stateMachine.submitEvent("NavigationComponent.CheckNearToPoi.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NavigationComponent.CheckNearToPoi.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("NavigationComponent.CheckNearToPoi.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NavigationComponent.CheckNearToPoi.Return");
                }
            }
        }
    });

	m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
		RCLCPP_INFO(m_node->get_logger(), "GoToCurrentPoiSkill::tickReturn %s", event.data().toMap()["result"].toString().toStdString().c_str());
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

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void GoToCurrentPoiSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
                                std::shared_ptr<bt_interfaces::srv::TickAction::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "GoToCurrentPoiSkill::tick");
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
    RCLCPP_INFO(m_node->get_logger(), "GoToCurrentPoiSkill::tickDone");
   
    response->is_ok = true;
}

