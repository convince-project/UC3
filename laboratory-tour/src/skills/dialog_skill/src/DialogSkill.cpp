/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "DialogSkill.h"
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

DialogSkill::DialogSkill(std::string name ) :
		m_name(std::move(name))
{
}

void DialogSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool DialogSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "DialogSkill::start");
	std::cout << "DialogSkill::start";

	m_tickService = m_node->create_service<bt_interfaces::srv::TickAction>(m_name + "Skill/tick",
                                                                           	std::bind(&DialogSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));

	m_haltService = m_node->create_service<bt_interfaces::srv::HaltAction>(m_name + "Skill/halt",
                                                                            	std::bind(&DialogSkill::halt,
                                                                            	this,
                                                                            	std::placeholders::_1,
                                                                            	std::placeholders::_2));

    m_stateMachine.connectToEvent("DialogComponent.EnableDialog.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeEnableDialog = rclcpp::Node::make_shared(m_name + "SkillNodeEnableDialog");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::EnableDialog>> clientEnableDialog = nodeEnableDialog->create_client<dialog_interfaces::srv::EnableDialog>("/DialogComponent/EnableDialog");
        auto request = std::make_shared<dialog_interfaces::srv::EnableDialog::Request>();
        auto eventParams = event.data().toMap();
        request->enable = convert<decltype(request->enable)>(eventParams["enable"].toString().toStdString());
        bool wait_succeded{true};
        while (!clientEnableDialog->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'EnableDialog'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("DialogComponent.EnableDialog.Return");
            } 
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientEnableDialog->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeEnableDialog, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    m_stateMachine.submitEvent("DialogComponent.EnableDialog.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.EnableDialog.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.EnableDialog.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.EnableDialog.Return");
                }
            }
        }
    });

	m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
		RCLCPP_INFO(m_node->get_logger(), "DialogSkill::tickReturn %s", event.data().toMap()["result"].toString().toStdString().c_str());
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

    m_stateMachine.connectToEvent("DialogComponent.GetState.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeGetState = rclcpp::Node::make_shared(m_name + "SkillNodeGetState");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::GetState>> clientGetState = nodeGetState->create_client<dialog_interfaces::srv::GetState>("/DialogComponent/GetState");
        auto request = std::make_shared<dialog_interfaces::srv::GetState::Request>();
        bool wait_succeded{true};
        while (!clientGetState->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GetState'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("DialogComponent.GetState.Return");
            } 
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientGetState->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeGetState, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    data.insert("state", response->state);
                    m_stateMachine.submitEvent("DialogComponent.GetState.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.GetState.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.GetState.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.GetState.Return");
                }
            }
        }
    });

	m_stateMachine.connectToEvent("HALT_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
		RCLCPP_INFO(m_node->get_logger(), "DialogSkill::haltresponse");
		m_haltResult.store(true);
	});

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void DialogSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
                                std::shared_ptr<bt_interfaces::srv::TickAction::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "DialogSkill::tick");
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
    RCLCPP_INFO(m_node->get_logger(), "DialogSkill::tickDone");
   
    response->is_ok = true;
}

void DialogSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::HaltAction::Request> request,
    [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::HaltAction::Response> response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "DialogSkill::halt");
    m_haltResult.store(false); //here we can put a struct
    m_stateMachine.submitEvent("CMD_HALT");
   
    while(!m_haltResult.load()) 
    {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
        // qInfo() <<  "active names" << m_stateMachine.activeStateNames();
    }
    RCLCPP_INFO(m_node->get_logger(), "DialogSkill::haltDone");
   
    response->is_ok = true;
}
