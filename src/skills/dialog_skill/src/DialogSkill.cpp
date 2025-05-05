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



    m_stateMachine.connectToEvent("DialogComponent.WaitForInteraction.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeWaitForInteraction = rclcpp::Node::make_shared(m_name + "SkillNodeWaitForInteraction");

        // add the client to remember the interactions
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::WaitForInteraction>> clientWaitForInteraction = nodeWaitForInteraction->create_client<dialog_interfaces::srv::WaitForInteraction>("/DialogComponent/WaitForInteraction");

        auto request = std::make_shared<dialog_interfaces::srv::WaitForInteraction::Request>();
        // get the optional input from keyboard and save the interaction into request->keyboard_interaction;
        auto eventParams = event.data().toMap();
        request->is_beginning_of_conversation = convert<decltype(request->is_beginning_of_conversation)>(eventParams["is_beginning_of_conversation"].toString().toStdString());
        std::cout << "[OPTIONAL] Please enter the interaction from keyboard: ";
        getline (std::cin, request->keyboard_interaction);
        bool wait_succeded{true};
        while (!clientWaitForInteraction->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'WaitForInteraction'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("DialogComponent.WaitForInteraction.Return");
            }
            else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for the service 'WaitForInteraction' to be available...");
            }
        }
        if (wait_succeded) {
            // send the request
            auto result = clientWaitForInteraction->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeWaitForInteraction, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok == true) {
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    data.insert("interaction", response->interaction.c_str());
                    m_stateMachine.submitEvent("DialogComponent.WaitForInteraction.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.WaitForInteraction.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.WaitForInteraction.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.WaitForInteraction.Return Failure");
                }
            }
        }
    });

    m_stateMachine.connectToEvent("SchedulerComponent.GetCurrentLanguage.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeGetCurrentLanguage = rclcpp::Node::make_shared(m_name + "SkillNodeGetCurrentLanguage");
        std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::GetCurrentLanguage>> clientGetCurrentLanguage = nodeGetCurrentLanguage->create_client<scheduler_interfaces::srv::GetCurrentLanguage>("/SchedulerComponent/GetCurrentLanguage");
        auto request = std::make_shared<scheduler_interfaces::srv::GetCurrentLanguage::Request>();
        bool wait_succeded{true};
        while (!clientGetCurrentLanguage->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GetCurrentLanguage'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("SchedulerComponent.GetCurrentLanguage.Return");
            }
            else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for the service 'SchedulerComponent/GetCurrentLanguage' to be available...");
            }
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientGetCurrentLanguage->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeGetCurrentLanguage, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    data.insert("language", response->language.c_str());
                    m_stateMachine.submitEvent("SchedulerComponent.GetCurrentLanguage.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.GetCurrentLanguage.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("SchedulerComponent.GetCurrentLanguage.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.GetCurrentLanguage.Return");
                }
            }
        }
    });

    m_stateMachine.connectToEvent("DialogComponent.SetLanguage.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeSetLanguage = rclcpp::Node::make_shared(m_name + "SkillNodeSetLanguage");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::SetLanguage>> clientSetLanguage = nodeSetLanguage->create_client<dialog_interfaces::srv::SetLanguage>("/DialogComponent/SetLanguage");
        auto request = std::make_shared<dialog_interfaces::srv::SetLanguage::Request>();
        auto eventParams = event.data().toMap();
        request->new_language = convert<decltype(request->new_language)>(eventParams["new_language"].toString().toStdString());
        bool wait_succeded{true};
        while (!clientSetLanguage->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'SetLanguage'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("DialogComponent.SetLanguage.Return");
            } 
        }
        if (wait_succeded) {
            // send the request
            auto result = clientSetLanguage->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeSetLanguage, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    m_stateMachine.submitEvent("DialogComponent.SetLanguage.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.SetLanguage.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.SetLanguage.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.SetLanguage.Return");
                }
            }
        }
    });

    m_stateMachine.connectToEvent("DialogComponent.CheckDuplicate.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeCheckDuplicate = rclcpp::Node::make_shared(m_name + "SkillCheckDuplicate");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::RememberInteractions>> clientCheckDuplicate = nodeCheckDuplicate->create_client<dialog_interfaces::srv::RememberInteractions>("/DialogComponent/RememberInteractions");
        auto request = std::make_shared<dialog_interfaces::srv::RememberInteractions::Request>();
        auto eventParams = event.data().toMap();
        request->interaction = convert<decltype(request->interaction)>(eventParams["interaction"].toString().toStdString());
        bool is_beginning_of_conversation = convert<decltype(request->is_beginning_of_conversation)>(eventParams["is_beginning_of_conversation"].toString().toStdString());
        std::cout << "Is beginning of conversation: " << is_beginning_of_conversation << std::endl;
        request->is_beginning_of_conversation = is_beginning_of_conversation;
        bool wait_succeded{true};
        while (!clientCheckDuplicate->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'CheckDuplicate'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("DialogComponent.CheckDuplicate.Return");
            } 
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientCheckDuplicate->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeCheckDuplicate, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS)
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    
                    data.insert("result", "SUCCESS");
                    // insert the duplicate index in int64 format
                    data.insert("duplicateIndex", response->index);
                    m_stateMachine.submitEvent("DialogComponent.CheckDuplicate.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.CheckDuplicate.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.CheckDuplicate.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.CheckDuplicate.Return");
                }
            }
        }
    });


    m_stateMachine.connectToEvent("DialogComponent.ShortenAndSpeak.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeShortenAndSpeak = rclcpp::Node::make_shared(m_name + "SkillShortenAndSpeak");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::ShortenAndSpeak>> clientShortenAndSpeak = nodeShortenAndSpeak->create_client<dialog_interfaces::srv::ShortenAndSpeak>("/DialogComponent/ShortenAndSpeak");
        auto request = std::make_shared<dialog_interfaces::srv::ShortenAndSpeak::Request>();
        auto eventParams = event.data().toMap();
        request->duplicate_index = convert<decltype(request->duplicate_index)>(eventParams["index"].toString().toStdString());
        std::cout << "Request: " << request->duplicate_index << std::endl;
        bool wait_succeded{true};
        while (!clientShortenAndSpeak->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'ShortenAndSpeak'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("DialogComponent.ShortenAndSpeak.Return");
            } 
        }
        // std::cout << "DialogComponent::ShortenAndSpeak.Call 2" << std::endl;
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientShortenAndSpeak->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeShortenAndSpeak, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    
                    data.insert("result", "SUCCESS");
                    m_stateMachine.submitEvent("DialogComponent.ShortenAndSpeak.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.ShortenAndSpeak.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.ShortenAndSpeak.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.ShortenAndSpeak.Return");
                }
            }
        }
    });

    m_stateMachine.connectToEvent("DialogComponent.Interpret.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::cout << "DialogComponent::Interpret.Call 0" << std::endl;
        std::shared_ptr<rclcpp::Node> nodeInterpret = rclcpp::Node::make_shared(m_name + "SkillInterpret");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::Interpret>> clientInterpret = nodeInterpret->create_client<dialog_interfaces::srv::Interpret>("/DialogComponent/Interpret");
        auto request = std::make_shared<dialog_interfaces::srv::Interpret::Request>();
        bool wait_succeded{true};
        // std::cout << "DialogComponent::Interpret.Call 1" << std::endl;
        while (!clientInterpret->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'Interpret'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("DialogComponent.Interpet.Return");
            } 
        }
        // std::cout << "DialogComponent::Interpret.Call 2" << std::endl;
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientInterpret->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeInterpret, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    
                    data.insert("result", "SUCCESS");
                    data.insert("isQuestion", response->is_question);
                    m_stateMachine.submitEvent("DialogComponent.Interpret.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.Interpret.Return");
                } else {

                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.Interpret.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.Interpret.Return");
                }
            }
        }
    });

    m_stateMachine.connectToEvent("DialogComponent.AnswerAndSpeak.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeAnswerAndSpeak = rclcpp::Node::make_shared(m_name + "SkillAnswerAndSpeak");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::AnswerAndSpeak>> clientAnswerAndSpeak = nodeAnswerAndSpeak->create_client<dialog_interfaces::srv::AnswerAndSpeak>("/DialogComponent/AnswerAndSpeak");
        auto request = std::make_shared<dialog_interfaces::srv::AnswerAndSpeak::Request>();
        bool wait_succeded{true};
        // std::cout << "DialogComponent::AnswerAndSpeak.Call 1" << std::endl;
        while (!clientAnswerAndSpeak->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'AnswerAndSpeak'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("DialogComponent.AnswerAndSpeak.Return");
            } 
        }
        // std::cout << "DialogComponent::AnswerAndSpeak.Call 2" << std::endl;
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientAnswerAndSpeak->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeAnswerAndSpeak, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    
                    data.insert("result", "SUCCESS");
                    m_stateMachine.submitEvent("DialogComponent.AnswerAndSpeak.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.AnswerAndSpeak.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.AnswerAndSpeak.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.AnswerAndSpeak.Return");
                }
            }
        }
    });

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
