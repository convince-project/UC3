/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "SayFollowMeSkill.h"
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

SayFollowMeSkill::SayFollowMeSkill(std::string name ) :
		m_name(std::move(name))
{
}

void SayFollowMeSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool SayFollowMeSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "SayFollowMeSkill::start");
	std::cout << "SayFollowMeSkill::start";

	m_tickService = m_node->create_service<bt_interfaces::srv::TickAction>(m_name + "Skill/tick",
                                                                           	std::bind(&SayFollowMeSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));

	m_haltService = m_node->create_service<bt_interfaces::srv::HaltAction>(m_name + "Skill/halt",
                                                                            	std::bind(&SayFollowMeSkill::halt,
                                                                            	this,
                                                                            	std::placeholders::_1,
                                                                            	std::placeholders::_2));

    m_stateMachine.connectToEvent("SchedulerComponent.SetCommand.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeSetCommand = rclcpp::Node::make_shared(m_name + "SkillNodeSetCommand");
        std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::SetCommand>> clientSetCommand = nodeSetCommand->create_client<scheduler_interfaces::srv::SetCommand>("/SchedulerComponent/SetCommand");
        auto request = std::make_shared<scheduler_interfaces::srv::SetCommand::Request>();
        auto eventParams = event.data().toMap();
        request->command = convert<decltype(request->command)>(eventParams["command"].toString().toStdString());
        std::cout << "Request" << request->command << std::endl;
        bool wait_succeded{true};
        while (!clientSetCommand->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'SetCommand'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("SchedulerComponent.SetCommand.Return");
            } 
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientSetCommand->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeSetCommand, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    m_stateMachine.submitEvent("SchedulerComponent.SetCommand.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.SetCommand.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("SchedulerComponent.SetCommand.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.SetCommand.Return");
                }
            }
        }
    });

    m_stateMachine.connectToEvent("SchedulerComponent.GetCurrentAction.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeGetCurrentAction = rclcpp::Node::make_shared(m_name + "SkillNodeGetCurrentAction");
        std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::GetCurrentAction>> clientGetCurrentAction = nodeGetCurrentAction->create_client<scheduler_interfaces::srv::GetCurrentAction>("/SchedulerComponent/GetCurrentAction");
        auto request = std::make_shared<scheduler_interfaces::srv::GetCurrentAction::Request>();
        bool wait_succeded{true};
        while (!clientGetCurrentAction->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GetCurrentAction'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("SchedulerComponent.GetCurrentAction.Return");
            } 
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientGetCurrentAction->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeGetCurrentAction, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    data.insert("param", response->param.c_str());
                    std::cout << "Param" << response->param.c_str() << std::endl;
                    m_stateMachine.submitEvent("SchedulerComponent.GetCurrentAction.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.GetCurrentAction.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("SchedulerComponent.GetCurrentAction.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.GetCurrentAction.Return");
                }
            }
        }
    });

    m_stateMachine.connectToEvent("TextToSpeechComponent.Speak.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeSpeak = rclcpp::Node::make_shared(m_name + "SkillNodeSpeak");
        std::shared_ptr<rclcpp::Client<text_to_speech_interfaces::srv::Speak>> clientSpeak = nodeSpeak->create_client<text_to_speech_interfaces::srv::Speak>("/TextToSpeechComponent/Speak");
        auto request = std::make_shared<text_to_speech_interfaces::srv::Speak::Request>();
        auto eventParams = event.data().toMap();
        request->text = convert<decltype(request->text)>(eventParams["text"].toString().toStdString());
        bool wait_succeded{true};
        while (!clientSpeak->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'Speak'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("TextToSpeechComponent.Speak.Return");
            } 
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientSpeak->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeSpeak, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    m_stateMachine.submitEvent("TextToSpeechComponent.Speak.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TextToSpeechComponent.Speak.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("TextToSpeechComponent.Speak.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TextToSpeechComponent.Speak.Return");
                }
            }
        }
    });

    m_stateMachine.connectToEvent("TextToSpeechComponent.IsSpeaking.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeIsSpeaking = rclcpp::Node::make_shared(m_name + "SkillNodeIsSpeaking");
        std::shared_ptr<rclcpp::Client<text_to_speech_interfaces::srv::IsSpeaking>> clientIsSpeaking = nodeIsSpeaking->create_client<text_to_speech_interfaces::srv::IsSpeaking>("/TextToSpeechComponent/IsSpeaking");
        auto request = std::make_shared<text_to_speech_interfaces::srv::IsSpeaking::Request>();
        bool wait_succeded{true};
        while (!clientIsSpeaking->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'IsSpeaking'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("TextToSpeechComponent.IsSpeaking.Return");
            } 
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientIsSpeaking->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeIsSpeaking, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    data.insert("is_speaking", response->is_speaking);
                    m_stateMachine.submitEvent("TextToSpeechComponent.IsSpeaking.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TextToSpeechComponent.IsSpeaking.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("TextToSpeechComponent.IsSpeaking.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TextToSpeechComponent.IsSpeaking.Return");
                }
            }
        }
    });

	m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
		RCLCPP_INFO(m_node->get_logger(), "SayFollowMeSkill::tickReturn %s", event.data().toMap()["result"].toString().toStdString().c_str());
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
		RCLCPP_INFO(m_node->get_logger(), "SayFollowMeSkill::haltresponse");
		m_haltResult.store(true);
	});

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void SayFollowMeSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
                                std::shared_ptr<bt_interfaces::srv::TickAction::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "SayFollowMeSkill::tick");
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
    RCLCPP_INFO(m_node->get_logger(), "SayFollowMeSkill::tickDone");
   
    response->is_ok = true;
}

void SayFollowMeSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::HaltAction::Request> request,
    [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::HaltAction::Response> response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "SayFollowMeSkill::halt");
    m_haltResult.store(false); //here we can put a struct
    m_stateMachine.submitEvent("CMD_HALT");
   
    while(!m_haltResult.load()) 
    {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
        // qInfo() <<  "active names" << m_stateMachine.activeStateNames();
    }
    RCLCPP_INFO(m_node->get_logger(), "SayFollowMeSkill::haltDone");
   
    response->is_ok = true;
}
