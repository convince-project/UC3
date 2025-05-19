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
#include "DialogSkillSM.h"
#include <bt_interfaces/msg/action_response.hpp>
#include <scheduler_interfaces/srv/get_current_language.hpp>

#include <dialog_interfaces/srv/manage_context.hpp>
#include <bt_interfaces/srv/tick_action.hpp>
#include <bt_interfaces/srv/halt_action.hpp>

#include <dialog_interfaces/srv/remember_interactions.hpp> // auto-generated from the .srv file
#include <dialog_interfaces/srv/wait_for_interaction.hpp>
#include <dialog_interfaces/srv/shorten_and_speak.hpp>
#include <dialog_interfaces/srv/answer_and_speak.hpp>

#include <text_to_speech_interfaces/srv/set_microphone.hpp>

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
	void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
			   std::shared_ptr<bt_interfaces::srv::TickAction::Response>      response);
	void halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::HaltAction::Request> request,
			   [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::HaltAction::Response> response);

private:
	void EnableMicrophone();
    void DisableMicrophone();


	std::shared_ptr<std::thread> m_threadSpin;
	std::shared_ptr<rclcpp::Node> m_node;
	std::mutex m_requestMutex;
	std::string m_name;
	DialogSkillAction m_stateMachine;
	
	std::atomic<Status> m_tickResult{Status::undefined};
	rclcpp::Service<bt_interfaces::srv::TickAction>::SharedPtr m_tickService;
	std::atomic<bool> m_haltResult{false};
	rclcpp::Service<bt_interfaces::srv::HaltAction>::SharedPtr m_haltService;
	
};

