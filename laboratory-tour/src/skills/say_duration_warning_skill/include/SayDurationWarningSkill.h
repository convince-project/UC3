# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "SayDurationWarningSkillSM.h"
#include <bt_interfaces/msg/action_response.hpp>
#include <text_to_speech_interfaces/srv/speak.hpp> 
#include <text_to_speech_interfaces/srv/is_speaking.hpp> 
#include <scheduler_interfaces/srv/set_command.hpp> 
#include <scheduler_interfaces/srv/get_current_action.hpp> 
#include <blackboard_interfaces/srv/set_int_blackboard.hpp> 
#include <blackboard_interfaces/srv/get_int_blackboard.hpp> 


#include <bt_interfaces/srv/tick_action.hpp>
#include <bt_interfaces/srv/halt_action.hpp>


#define SERVICE_TIMEOUT 8

enum class Status{
	undefined,
	running, 
	success,
	failure
};

class SayDurationWarningSkill
{
public:
	SayDurationWarningSkill(std::string name );
	bool start(int argc, char * argv[]);
	static void spin(std::shared_ptr<rclcpp::Node> node);
	
	void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
			   std::shared_ptr<bt_interfaces::srv::TickAction::Response>      response);
	
	void halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::HaltAction::Request> request,
			   [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::HaltAction::Response> response);

private:
	std::shared_ptr<std::thread> m_threadSpin;
	std::shared_ptr<rclcpp::Node> m_node;
	std::mutex m_requestMutex;
	std::string m_name;
	SayDurationWarningSkillAction m_stateMachine;
	std::atomic<Status> m_tickResult{Status::undefined};
	rclcpp::Service<bt_interfaces::srv::TickAction>::SharedPtr m_tickService;
	std::atomic<bool> m_haltResult{false};
	rclcpp::Service<bt_interfaces::srv::HaltAction>::SharedPtr m_haltService;
	
};

