# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "ResetSkillSM.h"
#include <bt_interfaces/msg/action_response.hpp>
#include <scheduler_interfaces/srv/set_poi.hpp> 
#include <blackboard_interfaces/srv/set_int_blackboard.hpp> 



#include <bt_interfaces/srv/tick_action.hpp>



#define SERVICE_TIMEOUT 8

enum class Status{
	undefined,
	running, 
	success,
	failure
};

class ResetSkill
{
public:
	ResetSkill(std::string name );
	bool start(int argc, char * argv[]);
	static void spin(std::shared_ptr<rclcpp::Node> node);
	
	void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
			   std::shared_ptr<bt_interfaces::srv::TickAction::Response>      response);
	
	
	

private:
	std::shared_ptr<std::thread> m_threadSpin;
	std::shared_ptr<rclcpp::Node> m_node;
	std::mutex m_requestMutex;
	std::string m_name;
	ResetSkillAction m_stateMachine;
	std::atomic<Status> m_tickResult{Status::undefined};
	rclcpp::Service<bt_interfaces::srv::TickAction>::SharedPtr m_tickService;
	
	
	
	
	
};

