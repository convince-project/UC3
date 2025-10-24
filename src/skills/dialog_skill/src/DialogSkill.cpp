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
#include <QStringList>

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

    nodeSpeak = rclcpp::Node::make_shared(m_name + "SkillNodeSpeak");
    this->clientSpeak =
        rclcpp_action::create_client<dialog_interfaces::action::Speak>(this->nodeSpeak, "/DialogComponent/SpeakAction");

    nodeSynthesizeText = rclcpp::Node::make_shared(m_name + "SkillNodeSynthesizeText");
    this->clientSynthesizeText =
        rclcpp_action::create_client<text_to_speech_interfaces::action::BatchGeneration>(this->nodeSynthesizeText, "/TextToSpeechComponent/BatchGenerationAction");

    m_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    m_executor->add_node(nodeWaitForInteraction);

    m_SpeakExecutor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    m_SpeakExecutor->add_node(nodeSpeak);

    m_SynthesizeTextExecutor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    m_SynthesizeTextExecutor->add_node(nodeSynthesizeText);

    // WaitForInteraction action fragment of code start

    m_stateMachine.connectToEvent("DialogComponent.WaitForInteraction.Call", [this]([[maybe_unused]] const QScxmlEvent &event)

                                  {

        std::cout << "Starting WaitForInteractionThread" << std::endl;
        EnableMicrophone();
        std::cout << "ENABLE MICROPHONE DONE" << std::endl;

        m_thread = QThread::create([event, this]() {

            if (!clientWaitForInteraction->wait_for_action_server())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Action server not available after waiting");
                rclcpp::shutdown();
            }

            auto goal_msg = dialog_interfaces::action::WaitForInteraction::Goal();
            auto eventParams = event.data().toMap();
            goal_msg.is_beginning_of_conversation = convert<decltype(goal_msg.is_beginning_of_conversation)>(eventParams["is_beginning_of_conversation"].toString().toStdString());
            // std::cout << "[OPTIONAL] Please enter the interaction from keyboard: ";
            // getline (std::cin, goal_msg.keyboard_interaction);

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

        m_thread->start(); });

    // WaitForInteraction action fragment of code end

    m_stateMachine.connectToEvent("DialogComponent.ManageContext.Call", [this]([[maybe_unused]] const QScxmlEvent &event)
                                  {
        std::shared_ptr<rclcpp::Node> nodeManageContext = rclcpp::Node::make_shared(m_name + "SkillManageContext");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::ManageContext>> clientManageContext = nodeManageContext->create_client<dialog_interfaces::srv::ManageContext>("/DialogComponent/ManageContext");
        auto request = std::make_shared<dialog_interfaces::srv::ManageContext::Request>();
        bool wait_succeded{true};
        while (!clientManageContext->wait_for_service(std::chrono::milliseconds(100))) {
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
                    m_LlmGeneratedDance = response->dance.c_str();
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
        bool wait_succeded{true};
        while (!clientSetLanguage->wait_for_service(std::chrono::milliseconds(100))) {
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
        while (!clientCheckDuplicate->wait_for_service(std::chrono::milliseconds(100))) {
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

    m_stateMachine.connectToEvent("DialogComponent.ShortenReply.Call", [this]([[maybe_unused]] const QScxmlEvent &event)
                                  {
        std::shared_ptr<rclcpp::Node> nodeShortenReply = rclcpp::Node::make_shared(m_name + "SkillShortenReply");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::ShortenReply>> clientShortenReply = nodeShortenReply->create_client<dialog_interfaces::srv::ShortenReply>("/DialogComponent/ShortenReply");
        auto request = std::make_shared<dialog_interfaces::srv::ShortenReply::Request>();
        auto eventParams = event.data().toMap();
        request->duplicate_index = convert<decltype(request->duplicate_index)>(eventParams["index"].toString().toStdString());
        request->context = convert<decltype(request->context)>(eventParams["context"].toString().toStdString());
        request->interaction = convert<decltype(request->interaction)>(eventParams["interaction"].toString().toStdString());
        std::cout << "Request: " << request->duplicate_index << std::endl;
        bool wait_succeded{true};
        while (!clientShortenReply->wait_for_service(std::chrono::milliseconds(100))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'ShortenReply'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("DialogComponent.ShortenReply.Return");
            } 
        }
        // std::cout << "DialogComponent::ShortenReply.Call 2" << std::endl;
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientShortenReply->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeShortenReply, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    
                    data.insert("result", "SUCCESS");

                    QStringList replies_qt;

                    for (const auto & reply : response->reply) {
                        replies_qt << QString::fromStdString(reply);
                    }

                    data.insert("reply", replies_qt.join("#").toStdString().c_str());

                    for (const auto& reply : response->reply) {
                        std::cout << "Reply: " << reply << std::endl;
                        m_replies.push_back(std::pair(reply, m_LlmGeneratedDance)); // use the dance generated by the LLM
                    }

                    m_stateMachine.submitEvent("DialogComponent.ShortenReply.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.ShortenReply.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.ShortenReply.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.ShortenReply.Return");
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
        while (!clientInterpretCommand->wait_for_service(std::chrono::milliseconds(100))) {
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

                    QStringList replies_qt;
                    QStringList dances_qt;

                    for (int i = 0; i < response->reply.size(); i++) {
                        std::cout << "Reply: " << response->reply[i] << " with dance: " << response->dance[i] << std::endl;
                        replies_qt << QString::fromStdString(response->reply[i]);
                        dances_qt << QString::fromStdString(response->dance[i]);
                        m_replies.push_back(std::pair(response->reply[i], response->dance[i]));
                    }

                    data.insert("reply", replies_qt.join("#").toStdString().c_str());
                    data.insert("dance", dances_qt.join("#").toStdString().c_str());
                    
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

    m_stateMachine.connectToEvent("DialogComponent.Answer.Call", [this]([[maybe_unused]] const QScxmlEvent &event)
                                  {
        std::shared_ptr<rclcpp::Node> nodeAnswer = rclcpp::Node::make_shared(m_name + "SkillAnswer");
        std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::Answer>> clientAnswer = nodeAnswer->create_client<dialog_interfaces::srv::Answer>("/DialogComponent/Answer");
        auto request = std::make_shared<dialog_interfaces::srv::Answer::Request>();
        auto eventParams = event.data().toMap();
        request->interaction = convert<decltype(request->interaction)>(eventParams["interaction"].toString().toStdString());
        request->context = convert<decltype(request->context)>(eventParams["context"].toString().toStdString());
        bool wait_succeded{true};
        // std::cout << "DialogComponent::Answer.Call 1" << std::endl;
        while (!clientAnswer->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'Answer'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("DialogComponent.Answer.Return");
            } 
        }
        // std::cout << "DialogComponent::Answer.Call 2" << std::endl;
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientAnswer->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeAnswer, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    
                    data.insert("result", "SUCCESS");

                    QStringList replies_qt;
                    for (const auto & reply : response->reply) {
                        replies_qt << QString::fromStdString(reply);
                        m_replies.push_back(std::pair(reply, m_LlmGeneratedDance)); // use the dance generated by the LLM
                    }

                    data.insert("reply", replies_qt.join("#").toStdString().c_str());
                    
                    m_stateMachine.submitEvent("DialogComponent.Answer.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.Answer.Return");
                } else {
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.Answer.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.Answer.Return");
                }
            }
        } });

    m_stateMachine.connectToEvent("TextToSpeechComponent.SynthesizeText.Call", [this]([[maybe_unused]] const QScxmlEvent &event)
                                  {

        std::cout << "Starting SynthesizeTextThread creation" << std::endl;                            
        m_SynthesizeTextThread = QThread::create([event, this]() {

            if (!clientSynthesizeText->wait_for_action_server())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "TextToSpeech Batch Generation Action server not available after waiting");
                rclcpp::shutdown();
            }

            auto goal_msg = text_to_speech_interfaces::action::BatchGeneration::Goal();
            std::vector<std::string> repliesText;

            for (const auto& reply : m_replies) {
                repliesText.push_back(reply.first);
            }

            goal_msg.texts = repliesText;

            for (auto &text : goal_msg.texts)
            {
                std::cout << "[TextToSpeechComponent::SynthesizeText] Text to be synthesized: " << text << std::endl;
            }

            auto send_goal_options = rclcpp_action::Client<text_to_speech_interfaces::action::BatchGeneration>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                [this](rclcpp_action::ClientGoalHandle<text_to_speech_interfaces::action::BatchGeneration>::SharedPtr goal_handle)
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
                [this](rclcpp_action::ClientGoalHandle<text_to_speech_interfaces::action::BatchGeneration>::SharedPtr,
                        const std::shared_ptr<const text_to_speech_interfaces::action::BatchGeneration::Feedback> feedback)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received feedback: %d", feedback->texts_left);
            };
            send_goal_options.result_callback =
                [this](const rclcpp_action::ClientGoalHandle<text_to_speech_interfaces::action::BatchGeneration>::WrappedResult &result)
            {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal succeeded");
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    m_stateMachine.submitEvent("TextToSpeechComponent.SynthesizeText.Return", data);
                }
                else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                    QVariantMap data;
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Goal was halted (canceled)");
                    data.insert("result", "HALTED");
                    m_stateMachine.submitEvent("TextToSpeechComponent.SynthesizeText.Return", data);
                } else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal failed");
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("TextToSpeechComponent.SynthesizeText.Return", data);
                }

                std::cout << "Cancel executor" << std::endl;
                m_SynthesizeTextExecutor->cancel();
                std::cout << "Executor cancelled" << std::endl;

            };
            std::cout << "Sending goal to TextToSpeech Batch Generation Action server" << std::endl;
            clientSynthesizeText->async_send_goal(goal_msg, send_goal_options);
            std::cout << "Goal sent" << std::endl;

            std::cout << "Spinning SynthesizeTextExecutor" << std::endl;
            m_SynthesizeTextExecutor->spin();
            std::cout << "SynthesizeTextExecutor spinned" << std::endl;
        });

        std::cout << "Starting SynthesizeTextThread" << std::endl;
        m_SynthesizeTextThread->start();
        std::cout << "SynthesizeTextThread started" << std::endl; });

    // Speak action fragment of code start

    m_stateMachine.connectToEvent("DialogComponent.Speak.Call", [this]([[maybe_unused]] const QScxmlEvent &event)

                                  {
            std::cout << "Starting SpeakThread creation" << std::endl;
                                    m_SpeakThread = QThread::create([event, this]() {

            if (!clientSpeak->wait_for_action_server())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Speak Action server not available after waiting");
                rclcpp::shutdown();
            }

            auto goal_msg = dialog_interfaces::action::Speak::Goal();

            std::vector<std::string> repliesText;
            std::vector<std::string> dances;

            for (const auto& [reply, dance] : m_replies) {
                std::cout << "Reply: " << reply << " Dance: " << dance << std::endl;
                repliesText.push_back(reply);
                dances.push_back(dance);
            }

            goal_msg.texts = repliesText;
            goal_msg.dances = dances;

            for (const auto &text : goal_msg.texts)
            {
                std::cout << "[DialogComponent::Speak] Text to be spoken: " << text << std::endl;
            }
            for (const auto &dance : goal_msg.dances)
            {
                std::cout << "[DialogComponent::Speak] Dance to be performed: " << dance << std::endl;
            }

            auto send_goal_options = rclcpp_action::Client<dialog_interfaces::action::Speak>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                [this](rclcpp_action::ClientGoalHandle<dialog_interfaces::action::Speak>::SharedPtr goal_handle)
            {
                if (!goal_handle)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Speak Goal was rejected by server");
                }
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speak Goal accepted by server, waiting for result");
                }
            };
            send_goal_options.feedback_callback =
                [this](rclcpp_action::ClientGoalHandle<dialog_interfaces::action::Speak>::SharedPtr,
                        const std::shared_ptr<const dialog_interfaces::action::Speak::Feedback> feedback)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speak Received feedback: %s", feedback->status.c_str());
            };
            send_goal_options.result_callback =
                [this](const rclcpp_action::ClientGoalHandle<dialog_interfaces::action::Speak>::WrappedResult &result)
            {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speak Goal succeeded");
                    QVariantMap data;
                    data.insert("result", "SUCCESS");
                    data.insert("is_reply_finished", result.result->is_reply_finished);
                    if (result.result->is_reply_finished) {
                        // Clear replies vector only if the whole reply has been spoken
                        std::cout << "Clearing replies vector" << std::endl;
                        m_replies.clear();
                    }
                    m_stateMachine.submitEvent("DialogComponent.Speak.Return", data);
                }
                else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                    QVariantMap data;
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Speak Goal was halted (canceled)");
                    m_replies.clear();
                    data.insert("result", "HALTED");
                    m_stateMachine.submitEvent("DialogComponent.Speak.Return", data);
                } else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Speak Goal failed");
                    QVariantMap data;
                    data.insert("result", "FAILURE");
                    m_stateMachine.submitEvent("DialogComponent.Speak.Return", data);
                }

                std::cout << "Cancelling Speak executor" << std::endl;
                // Stop executor if needed:
                m_SpeakExecutor->cancel();
                std::cout << "Executor cancelled" << std::endl;

            };
            std::cout << "Sending goal to Speak Action server" << std::endl;
            clientSpeak->async_send_goal(goal_msg, send_goal_options);
            std::cout << "Goal sent" << std::endl;

            std::cout << "Spinning SpeakExecutor" << std::endl;
            m_SpeakExecutor->spin();
            std::cout << "SpeakExecutor spinned" << std::endl;
        });

        std::cout << "Starting SpeakThread" << std::endl;
        m_SpeakThread->start();
        std::cout << "SpeakThread started" << std::endl; });

    // Speak action fragment of code end

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

    std::cout << "Cancelling all goals" << std::endl;
    clientWaitForInteraction->async_cancel_all_goals();
    std::cout << "wait for interaction goals cancelled" << std::endl;
    clientSynthesizeText->async_cancel_all_goals();
    std::cout << "synthesize text goals cancelled" << std::endl;
    clientSpeak->async_cancel_all_goals();
    std::cout << "speak goals cancelled" << std::endl;

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
    auto setCommandClientNode = rclcpp::Node::make_shared("DialogComponentSetCommandNode");

    auto setMicrophoneClient = setCommandClientNode->create_client<text_to_speech_interfaces::srv::SetMicrophone>("/TextToSpeechComponent/SetMicrophone");
    auto request = std::make_shared<text_to_speech_interfaces::srv::SetMicrophone::Request>();
    request->enabled = true;
    // Check for the presence of the service. Wait for it if not available
    while (!setMicrophoneClient->wait_for_service(std::chrono::milliseconds(100)))
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

// void DialogSkill::DisableMicrophone()
// {
//     // Setting the microphone off
//     auto setCommandClientNode = rclcpp::Node::make_shared("DialogComponentSetCommandNode");

//     auto setMicrophoneClient = setCommandClientNode->create_client<text_to_speech_interfaces::srv::SetMicrophone>("/TextToSpeechComponent/SetMicrophone");
//     auto request = std::make_shared<text_to_speech_interfaces::srv::SetMicrophone::Request>();
//     request->enabled = false;
//     // Wait for service
//     while (!setMicrophoneClient->wait_for_service(std::chrono::seconds(1)))
//     {
//         if (!rclcpp::ok())
//         {
//             RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setCommandClient'. Exiting.");
//         }
//     }
//     auto result = setMicrophoneClient->async_send_request(request);
//     // Wait for the result.
//     if (rclcpp::spin_until_future_complete(setCommandClientNode, result) == rclcpp::FutureReturnCode::SUCCESS)
//     {
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mic disabled");
//     }
//     else
//     {
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_microphone");
//     }
// }
