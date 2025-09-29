/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "DialogSkillSM.h"
#include <bt_interfaces_dummy/msg/action_response.hpp>
#include <scheduler_interfaces/srv/get_current_language.hpp>

#include <bt_interfaces_dummy/srv/tick_action.hpp>
#include <bt_interfaces_dummy/srv/halt_action.hpp>

#include <dialog_interfaces/srv/manage_context.hpp>
#include <dialog_interfaces/srv/remember_interactions.hpp> // auto-generated from the .srv file
#include <dialog_interfaces/srv/shorten_reply.hpp>
#include <dialog_interfaces/srv/answer.hpp>
#include <dialog_interfaces/srv/set_language.hpp>
#include <dialog_interfaces/srv/interpret_command.hpp>

#include <dialog_interfaces/action/speak.hpp>
#include <dialog_interfaces/action/wait_for_interaction.hpp>

#include <dialog_interfaces/srv/set_microphone.hpp>
#include <text_to_speech_interfaces/action/batch_generation.hpp>

#include <QThread>

enum class Status{
	undefined,
	running,
	success,
	failure
};

class DialogSkill
{
public:
	DialogSkill(std::string name );
	bool start(int argc, char * argv[]);
	static void spin(std::shared_ptr<rclcpp::Node> node);
	void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Request> request,
			   std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Response>      response);
	void halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Request> request,
			   [[maybe_unused]] std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Response> response);

	using ActionWaitForInteraction = dialog_interfaces::action::WaitForInteraction;
	using GoalHandleWaitForInteraction = rclcpp_action::ClientGoalHandle<ActionWaitForInteraction>;

	using ActionSpeak = dialog_interfaces::action::Speak;
	using GoalHandleSpeak = rclcpp_action::ClientGoalHandle<ActionSpeak>;

	using ActionSynthesizeText = text_to_speech_interfaces::action::BatchGeneration;
	using GoalHandleSynthesizeText = rclcpp_action::ClientGoalHandle<ActionSynthesizeText>;

private:
	void EnableMicrophone();
    void DisableMicrophone();


	std::shared_ptr<std::thread> m_threadSpin;
	std::shared_ptr<rclcpp::Node> m_node;
	std::mutex m_requestMutex;
	std::string m_name;
	DialogSkillAction m_stateMachine;
	
	std::atomic<Status> m_tickResult{Status::undefined};
	rclcpp::Service<bt_interfaces_dummy::srv::TickAction>::SharedPtr m_tickService;
	std::atomic<bool> m_haltResult{false};
	rclcpp::Service<bt_interfaces_dummy::srv::HaltAction>::SharedPtr m_haltService;

	std::shared_ptr<rclcpp::Node> nodeWaitForInteraction;
	std::shared_ptr<rclcpp_action::Client<dialog_interfaces::action::WaitForInteraction>> clientWaitForInteraction;

	std::shared_ptr<rclcpp::Node> nodeSpeak;
	std::shared_ptr<rclcpp_action::Client<dialog_interfaces::action::Speak>> clientSpeak;

	std::shared_ptr<rclcpp::Node> nodeSynthesizeText;
	std::shared_ptr<rclcpp_action::Client<text_to_speech_interfaces::action::BatchGeneration>> clientSynthesizeText;

	// Members
	std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
	QThread* m_thread = nullptr;

	std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> m_SpeakExecutor;
	QThread* m_SpeakThread = nullptr;

	std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> m_SynthesizeTextExecutor;
	QThread* m_SynthesizeTextThread = nullptr;

	// save the vector of text for the reply
	std::vector<std::string> m_replies;


};

