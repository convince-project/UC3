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

template <typename T>
T convert(const std::string &str)
{
    if constexpr (std::is_same_v<T, int>)
    {
        return std::stoi(str);
    }
    else if constexpr (std::is_same_v<T, double>)
    {
        return std::stod(str);
    }
    else if constexpr (std::is_same_v<T, float>)
    {
        return std::stof(str);
    }
    else if constexpr (std::is_same_v<T, bool>)
    {
        if (str == "true" || str == "1")
        {
            return true;
        }
        else if (str == "false" || str == "0")
        {
            return false;
        }
        else
        {
            throw std::invalid_argument("Invalid boolean value");
        }
    }
    else if constexpr (std::is_same_v<T, std::string>)
    {
        return str;
    }
    else
    {
        // Handle unsupported types
        throw std::invalid_argument("Unsupported type conversion");
    }
}

DialogSkill::DialogSkill(std::string name) : m_name(std::move(name))
{
}

void DialogSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::spin(node);
    rclcpp::shutdown();
}

bool DialogSkill::start(int argc, char *argv[])
{
    if (!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }

    m_node = rclcpp::Node::make_shared(m_name + "Skill");
    RCLCPP_DEBUG_STREAM(m_node->get_logger(), "DialogSkill::start");
    std::cout << "DialogSkill::start";

    m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickAction>(m_name + "Skill/tick",
                                                                           std::bind(&DialogSkill::tick,
                                                                                     this,
                                                                                     std::placeholders::_1,
                                                                                     std::placeholders::_2));

    m_haltService = m_node->create_service<bt_interfaces_dummy::srv::HaltAction>(m_name + "Skill/halt",
                                                                           std::bind(&DialogSkill::halt,
                                                                                     this,
                                                                                     std::placeholders::_1,
                                                                                     std::placeholders::_2));

    nodeWaitForInteraction = rclcpp::Node::make_shared(m_name + "SkillNodeWaitForInteraction");
    this->clientWaitForInteraction =
        rclcpp_action::create_client<dialog_interfaces::action::WaitForInteraction>(this->nodeWaitForInteraction, "/DialogComponent/WaitForInteractionAction");

    m_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    m_executor->add_node(nodeWaitForInteraction);

    // m_stateMachine.connectToEvent("DialogComponent.WaitForInteraction.Call", [this]([[maybe_unused]] const QScxmlEvent &event)
    //                               {
    //     std::shared_ptr<rclcpp::Node> nodeWaitForInteraction = rclcpp::Node::make_shared(m_name + "SkillNodeWaitForInteraction");

    //     // add the client to remember the interactions
    //     std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::WaitForInteraction>> clientWaitForInteraction = nodeWaitForInteraction->create_client<dialog_interfaces::srv::WaitForInteraction>("/DialogComponent/WaitForInteraction");

    //     auto request = std::make_shared<dialog_interfaces::srv::WaitForInteraction::Request>();
    //     // get the optional input from keyboard and save the interaction into request->keyboard_interaction;
    //     auto eventParams = event.data().toMap();
    //     request->is_beginning_of_conversation = convert<decltype(request->is_beginning_of_conversation)>(eventParams["is_beginning_of_conversation"].toString().toStdString());
    //     std::cout << "[OPTIONAL] Please enter the interaction from keyboard: ";
    //     getline (std::cin, request->keyboard_interaction);
    //     bool wait_succeded{true};
    //     while (!clientWaitForInteraction->wait_for_service(std::chrono::seconds(1))) {
    //         if (!rclcpp::ok()) {
    //             RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'WaitForInteraction'. Exiting.");
    //             wait_succeded = false;
    //             m_stateMachine.submitEvent("DialogComponent.WaitForInteraction.Return");
    //         }
    //         else {
    //             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for the service 'WaitForInteraction' to be available...");
    //         }
    //     }
    //     if (wait_succeded) {
    //         // send the request
    //         auto result = clientWaitForInteraction->async_send_request(request);
    //         auto futureResult = rclcpp::spin_until_future_complete(nodeWaitForInteraction, result);
    //         auto response = result.get();
    //         if (futureResult == rclcpp::FutureReturnCode::SUCCESS)
    //         {
    //             if( response->is_ok == true) {
    //                 QVariantMap data;
    //                 data.insert("result", "SUCCESS");
    //                 data.insert("interaction", response->interaction.c_str());
    //                 m_stateMachine.submitEvent("DialogComponent.WaitForInteraction.Return", data);
    //                 RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.WaitForInteraction.Return");
    //             } else {
    //                 QVariantMap data;
    //                 data.insert("result", "FAILURE");
    //                 m_stateMachine.submitEvent("DialogComponent.WaitForInteraction.Return", data);
    //                 RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.WaitForInteraction.Return Failure");
    //             }
    //         }
    //     } });

    // WaitForInteraction action fragment of code start

    m_stateMachine.connectToEvent("DialogComponent.WaitForInteraction.Call", [this]([[maybe_unused]] const QScxmlEvent &event)

        
                                  {
        m_thread = QThread::create([event, this]() {

            if (!clientWaitForInteraction->wait_for_action_server())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Action server not available after waiting");
                rclcpp::shutdown();
            }

            auto goal_msg = dialog_interfaces::action::WaitForInteraction::Goal();
            auto eventParams = event.data().toMap();
            goal_msg.is_beginning_of_conversation = convert<decltype(goal_msg.is_beginning_of_conversation)>(eventParams["is_beginning_of_conversation"].toString().toStdString());
            std::cout << "[OPTIONAL] Please enter the interaction from keyboard: ";
            getline (std::cin, goal_msg.keyboard_interaction);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.WaitForInteraction.Call received with is_beginning_of_conversation: %s and keyboard_interaction: %s",
                        goal_msg.is_beginning_of_conversation ? "true" : "false", goal_msg.keyboard_interaction.c_str());
            auto send_goal_options = rclcpp_action::Client<dialog_interfaces::action::WaitForInteraction>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                [this](rclcpp_action::ClientGoalHandle<dialog_interfaces::action::WaitForInteraction>::SharedPtr goal_handle)
            {
                if (!goal_handle)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was rejected by server");
                }
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted by server, waiting for result");
                }
            };
            send_goal_options.feedback_callback =
                [this](rclcpp_action::ClientGoalHandle<dialog_interfaces::action::WaitForInteraction>::SharedPtr,
                        const std::shared_ptr<const dialog_interfaces::action::WaitForInteraction::Feedback> feedback)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received feedback: %s", feedback->status.c_str());
            };
            send_goal_options.result_callback =
                [this](const rclcpp_action::ClientGoalHandle<dialog_interfaces::action::WaitForInteraction>::WrappedResult &result)
            {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal succeeded");
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    data.insert("interaction", QString::fromStdString(result.result->interaction));
                    m_stateMachine.submitEvent("DialogComponent.WaitForInteraction.Return", data);
                }
                else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                    QVariantMap data;
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Goal was halted (canceled)");
                    data.insert("result", "HALTED");
                    m_stateMachine.submitEvent("DialogComponent.WaitForInteraction.Return", data);
                } else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal failed");
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.WaitForInteraction.Return", data);
                }

                // Stop executor if needed:
                m_executor->cancel();

            };
            clientWaitForInteraction->async_send_goal(goal_msg, send_goal_options);

            m_executor->spin();
        });

        m_thread->start();
    });

    // WaitForInteraction action fragment of code end

    m_stateMachine.connectToEvent("DialogComponent.ManageContext.Call", [this]([[maybe_unused]] const QScxmlEvent &event)
                                  {
        std::shared_ptr<rclcpp::Node> nodeManageContext = rclcpp::Node::make_shared(m_name + "SkillManageContext");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::ManageContext>> clientManageContext = nodeManageContext->create_client<dialog_interfaces::srv::ManageContext>("/DialogComponent/ManageContext");
        auto request = std::make_shared<dialog_interfaces::srv::ManageContext::Request>();
        bool wait_succeded{true};
        while (!clientManageContext->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'ManageContext'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("DialogComponent.ManageContext.Return");
            } 
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientManageContext->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeManageContext, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS)
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    
                    data.insert("result", "SUCCESS");
                    // insert the duplicate index in int64 format
                    data.insert("language", response->language.c_str());
                    data.insert("context", response->context.c_str());
                    data.insert("isPoIEnded", response->is_poi_ended);
                    data.insert("dance", response->dance.c_str());
                    m_stateMachine.submitEvent("DialogComponent.ManageContext.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.ManageContext.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.ManageContext.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.ManageContext.Return");
                }
            }
        } });

    m_stateMachine.connectToEvent("DialogComponent.SetLanguage.Call", [this]([[maybe_unused]] const QScxmlEvent &event)
                                  {
        std::shared_ptr<rclcpp::Node> nodeSetLanguage = rclcpp::Node::make_shared(m_name + "SkillSetLanguage");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::SetLanguage>> clientSetLanguage = nodeSetLanguage->create_client<dialog_interfaces::srv::SetLanguage>("/DialogComponent/SetLanguage");
        auto request = std::make_shared<dialog_interfaces::srv::SetLanguage::Request>();
        auto eventParams = event.data().toMap();
        request->language = convert<decltype(request->language)>(eventParams["new_language"].toString().toStdString());
        std::cout << "DialogComponent::SetLanguage call received with language: " << request->language << std::endl;
        bool wait_succeded{true};
        while (!clientSetLanguage->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'SetLanguage'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("DialogComponent.SetLanguage.Return");
            }
            else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for the service 'SetLanguage' to be available...");
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
        } });

    m_stateMachine.connectToEvent("DialogComponent.CheckDuplicate.Call", [this]([[maybe_unused]] const QScxmlEvent &event)
                                  {
        std::shared_ptr<rclcpp::Node> nodeCheckDuplicate = rclcpp::Node::make_shared(m_name + "SkillCheckDuplicate");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::RememberInteractions>> clientCheckDuplicate = nodeCheckDuplicate->create_client<dialog_interfaces::srv::RememberInteractions>("/DialogComponent/RememberInteractions");
        auto request = std::make_shared<dialog_interfaces::srv::RememberInteractions::Request>();
        auto eventParams = event.data().toMap();
        request->interaction = convert<decltype(request->interaction)>(eventParams["interaction"].toString().toStdString());
        bool is_beginning_of_conversation = convert<decltype(request->is_beginning_of_conversation)>(eventParams["is_beginning_of_conversation"].toString().toStdString());
        std::cout << "Is beginning of conversation: " << is_beginning_of_conversation << std::endl;
        request->is_beginning_of_conversation = is_beginning_of_conversation;
        request->context = convert<decltype(request->context)>(eventParams["context"].toString().toStdString());
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
        } });

    m_stateMachine.connectToEvent("DialogComponent.ShortenAndSpeak.Call", [this]([[maybe_unused]] const QScxmlEvent &event)
                                  {
        std::shared_ptr<rclcpp::Node> nodeShortenAndSpeak = rclcpp::Node::make_shared(m_name + "SkillShortenAndSpeak");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::ShortenAndSpeak>> clientShortenAndSpeak = nodeShortenAndSpeak->create_client<dialog_interfaces::srv::ShortenAndSpeak>("/DialogComponent/ShortenAndSpeak");
        auto request = std::make_shared<dialog_interfaces::srv::ShortenAndSpeak::Request>();
        auto eventParams = event.data().toMap();
        request->duplicate_index = convert<decltype(request->duplicate_index)>(eventParams["index"].toString().toStdString());
        request->context = convert<decltype(request->context)>(eventParams["context"].toString().toStdString());
        request->interaction = convert<decltype(request->interaction)>(eventParams["interaction"].toString().toStdString());
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
                    data.insert("reply", QString::fromStdString(response->reply));
                    data.insert("is_reply_finished", response->is_reply_finished);
                    m_stateMachine.submitEvent("DialogComponent.ShortenAndSpeak.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.ShortenAndSpeak.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.ShortenAndSpeak.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.ShortenAndSpeak.Return");
                }
            }
        } });

    m_stateMachine.connectToEvent("DialogComponent.InterpretCommand.Call", [this]([[maybe_unused]] const QScxmlEvent &event)
                                  {
        std::cout << "DialogComponent::InterpretCommand.Call 0" << std::endl;
        std::shared_ptr<rclcpp::Node> nodeInterpretCommand = rclcpp::Node::make_shared(m_name + "SkillInterpretCommand");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::InterpretCommand>> clientInterpretCommand = nodeInterpretCommand->create_client<dialog_interfaces::srv::InterpretCommand>("/DialogComponent/InterpretCommand");
        auto request = std::make_shared<dialog_interfaces::srv::InterpretCommand::Request>();
        auto eventParams = event.data().toMap();
        request->context = convert<decltype(request->context)>(eventParams["context"].toString().toStdString());
        bool wait_succeded{true};
        // std::cout << "DialogComponent::InterpretCommand.Call 1" << std::endl;
        while (!clientInterpretCommand->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'InterpretCommand'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("DialogComponent.Interpet.Return");
            }
        }
        // std::cout << "DialogComponent::InterpretCommand.Call 2" << std::endl;
        if (wait_succeded) {
            // send the request
            auto result = clientInterpretCommand->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeInterpretCommand, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS)
            {
                if( response->is_ok ==true) {
                    QVariantMap data;

                    data.insert("result", "SUCCESS");
                    data.insert("reply", QString::fromStdString(response->reply));
                    data.insert("is_reply_finished", response->is_reply_finished);
                    m_stateMachine.submitEvent("DialogComponent.InterpretCommand.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.InterpretCommand.Return");
                } else {

                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.InterpretCommand.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.InterpretCommand.Return");
                }
            }
        } });

    m_stateMachine.connectToEvent("DialogComponent.AnswerAndSpeak.Call", [this]([[maybe_unused]] const QScxmlEvent &event)
                                  {
        std::shared_ptr<rclcpp::Node> nodeAnswerAndSpeak = rclcpp::Node::make_shared(m_name + "SkillAnswerAndSpeak");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::AnswerAndSpeak>> clientAnswerAndSpeak = nodeAnswerAndSpeak->create_client<dialog_interfaces::srv::AnswerAndSpeak>("/DialogComponent/AnswerAndSpeak");
        auto request = std::make_shared<dialog_interfaces::srv::AnswerAndSpeak::Request>();
        auto eventParams = event.data().toMap();
        request->interaction = convert<decltype(request->interaction)>(eventParams["interaction"].toString().toStdString());
        request->context = convert<decltype(request->context)>(eventParams["context"].toString().toStdString());
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
                    data.insert("reply", QString::fromStdString(response->reply));
                    data.insert("is_reply_finished", response->is_reply_finished);
                    m_stateMachine.submitEvent("DialogComponent.AnswerAndSpeak.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.AnswerAndSpeak.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.AnswerAndSpeak.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.AnswerAndSpeak.Return");
                }
            }
        } });

    m_stateMachine.connectToEvent("DialogComponent.Speak.Call", [this]([[maybe_unused]] const QScxmlEvent &event)
                                  {
        std::shared_ptr<rclcpp::Node> nodeSpeak = rclcpp::Node::make_shared(m_name + "SkillSpeak");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::Speak>> clientSpeak = nodeSpeak->create_client<dialog_interfaces::srv::Speak>("/DialogComponent/Speak");
        auto request = std::make_shared<dialog_interfaces::srv::Speak::Request>();
        auto eventParams = event.data().toMap();
        request->text = convert<decltype(request->text)>(eventParams["text"].toString().toStdString());
        request->dance = convert<decltype(request->dance)>(eventParams["dance"].toString().toStdString());
        bool wait_succeded{true};
        while (!clientSpeak->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'Speak'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("DialogComponent.Speak.Return");
            }
            else {  
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for the service 'Speak' to be available...");
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
                    m_stateMachine.submitEvent("DialogComponent.Speak.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.Speak.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.Speak.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.Speak.Return");
                }
            }
        } });

    m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]] const QScxmlEvent &event)
                                  {
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
		} });

    m_stateMachine.connectToEvent("HALT_RESPONSE", [this]([[maybe_unused]] const QScxmlEvent &event)
                                  {
		RCLCPP_INFO(m_node->get_logger(), "DialogSkill::haltresponse");
		m_haltResult.store(true); });

    m_stateMachine.start();
    m_threadSpin = std::make_shared<std::thread>(spin, m_node);

    return true;
}

void DialogSkill::tick([[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Request> request,
                       std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Response> response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "DialogSkill::tick");
    auto message = bt_interfaces_dummy::msg::ActionResponse();
    m_tickResult.store(Status::undefined); // here we can put a struct
    m_stateMachine.submitEvent("CMD_TICK");

    EnableMicrophone();

    while (m_tickResult.load() == Status::undefined)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // qInfo() <<  "active names" << m_stateMachine.activeStateNames();
    }

    // DisableMicrophone();

    switch (m_tickResult.load())
    {
    case Status::running:
        response->status = message.SKILL_RUNNING;
        break;
    case Status::failure:
        response->status = message.SKILL_FAILURE;
        break;
    case Status::success:
        response->status = message.SKILL_SUCCESS;
        break;
    }
    RCLCPP_INFO(m_node->get_logger(), "DialogSkill::tickDone");

    response->is_ok = true;
}

void DialogSkill::halt([[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Request> request,
                       [[maybe_unused]] std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Response> response)
{

    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "DialogSkill::halt");

    clientWaitForInteraction->async_cancel_all_goals();

    m_haltResult.store(false); // here we can put a struct
    m_stateMachine.submitEvent("CMD_HALT");


    while (!m_haltResult.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // qInfo() <<  "active names" << m_stateMachine.activeStateNames();
    }
    RCLCPP_INFO(m_node->get_logger(), "DialogSkill::haltDone");

    response->is_ok = true;
}

void DialogSkill::EnableMicrophone()
{
    // --------------------------Start Mic service call ----------------------
    auto setCommandClientNode = rclcpp::Node::make_shared("TextToSpeechComponentSetCommandNode");

    auto setMicrophoneClient = setCommandClientNode->create_client<text_to_speech_interfaces::srv::SetMicrophone>("/TextToSpeechComponent/SetMicrophone");
    auto request = std::make_shared<text_to_speech_interfaces::srv::SetMicrophone::Request>();
    request->enabled = true;
    // Check for the presence of the service. Wait for it if not available
    while (!setMicrophoneClient->wait_for_service(std::chrono::seconds(1)))
    {
        // While waiting, check if the node is still alive, otherwise exit
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setCommandClient'. Exiting.");
        }
    }
    // Send request and record the result from the service
    auto result = setMicrophoneClient->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(setCommandClientNode, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mic Enabled");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_microphone");
    }
}

void DialogSkill::DisableMicrophone()
{
    // Setting the microphone off
    auto setCommandClientNode = rclcpp::Node::make_shared("TextToSpeechComponentSetCommandNode");

    auto setMicrophoneClient = setCommandClientNode->create_client<text_to_speech_interfaces::srv::SetMicrophone>("/TextToSpeechComponent/SetMicrophone");
    auto request = std::make_shared<text_to_speech_interfaces::srv::SetMicrophone::Request>();
    request->enabled = false;
    // Wait for service
    while (!setMicrophoneClient->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setCommandClient'. Exiting.");
        }
    }
    auto result = setMicrophoneClient->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(setCommandClientNode, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mic disabled");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_microphone");
    }
}
