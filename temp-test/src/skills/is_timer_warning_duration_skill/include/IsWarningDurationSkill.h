# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "IsWarningDurationSkillSM.h"
#include <bt_interfaces_dummy/msg/conditioncondition_response.hpp>
#include <blackboard_interfaces/srv/get_int_blackboard.hpp> 



#include <bt_interfaces_dummy/srv/tick_conditioncondition.hpp>



#define SERVICE_TIMEOUT 8
#define SKILL_SUCCESS 0
#define SKILL_FAILURE 1
#define SKILL_RUNNING 2

enum class Status{
	undefined,
	success,
	failure
};

class IsWarningDurationSkill
{
public:
	IsWarningDurationSkill(std::string name );
	bool start(int argc, char * argv[]);
	static void spin(std::shared_ptr<rclcpp::Node> node);
	
	void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickConditionCondition::Request> request,
			   std::shared_ptr<bt_interfaces_dummy::srv::TickConditionCondition::Response>      response);
	
	
	

private:
	std::shared_ptr<std::thread> m_threadSpin;
	std::shared_ptr<rclcpp::Node> m_node;
	std::mutex m_requestMutex;
	std::string m_name;
	IsWarningDurationSkillConditionCondition m_stateMachine;
	std::atomic<Status> m_tickResult{Status::undefined};
	rclcpp::Service<bt_interfaces_dummy::srv::TickConditionCondition>::SharedPtr m_tickService;
	
	
	
	
	
	
	

};

