# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "SayWhileNavigatingSkillSM.h"
#include <bt_interfaces/msg/action_response.hpp>
#include <turn_back_manager_interfaces/srv/get_turn_backs_counter.hpp> 
#include <narrate_interfaces/srv/stop.hpp> 
#include <narrate_interfaces/srv/narrate.hpp> 
#include <narrate_interfaces/srv/is_done.hpp> 


#include <bt_interfaces/srv/tick_action.hpp>
#include <bt_interfaces/srv/halt_action.hpp>


#define SERVICE_TIMEOUT 8

enum class Status{
	undefined,
	running, 
	success,
	failure
};

class SayWhileNavigatingSkill
{
public:
	SayWhileNavigatingSkill(std::string name );
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
	SayWhileNavigatingSkillAction m_stateMachine;
	std::atomic<Status> m_tickResult{Status::undefined};
	rclcpp::Service<bt_interfaces::srv::TickAction>::SharedPtr m_tickService;
	std::atomic<bool> m_haltResult{false};
	rclcpp::Service<bt_interfaces::srv::HaltAction>::SharedPtr m_haltService;
	
};

