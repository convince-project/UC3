# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "RunTimerSkillSM.h"
#include <bt_interfaces_dummy/msg/condition_response.hpp>
#include <timer_check_for_people_interfaces/srv/start_timer.hpp> 
#include <timer_check_for_people_interfaces/srv/is_timer_active.hpp> 



#include <bt_interfaces_dummy/srv/tick_condition.hpp>



#define SERVICE_TIMEOUT 8
#define SKILL_SUCCESS 0
#define SKILL_FAILURE 1
#define SKILL_RUNNING 2

enum class Status{
	undefined,
	success,
	failure
};

class RunTimerSkill
{
public:
	RunTimerSkill(std::string name );
	bool start(int argc, char * argv[]);
	static void spin(std::shared_ptr<rclcpp::Node> node);
	
	void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickCondition::Request> request,
			   std::shared_ptr<bt_interfaces_dummy::srv::TickCondition::Response>      response);
	
	
	

private:
	std::shared_ptr<std::thread> m_threadSpin;
	std::shared_ptr<rclcpp::Node> m_node;
	std::mutex m_requestMutex;
	std::string m_name;
	RunTimerSkillCondition m_stateMachine;
	std::atomic<Status> m_tickResult{Status::undefined};
	rclcpp::Service<bt_interfaces_dummy::srv::TickCondition>::SharedPtr m_tickService;
	
	
	
	
	
	
	

};

