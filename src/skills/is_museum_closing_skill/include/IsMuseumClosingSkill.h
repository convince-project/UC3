# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "IsMuseumClosingSkillSM.h"
#include <bt_interfaces/msg/condition_response.hpp>
#include <time_interfaces/srv/is_museum_closing.hpp> 


#include <bt_interfaces/srv/tick_condition.hpp>



#define SERVICE_TIMEOUT 8

enum class Status{
	undefined,
	success,
	failure
};

class IsMuseumClosingSkill
{
public:
	IsMuseumClosingSkill(std::string name );
	bool start(int argc, char * argv[]);
	static void spin(std::shared_ptr<rclcpp::Node> node);
	
	void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickCondition::Request> request,
			   std::shared_ptr<bt_interfaces::srv::TickCondition::Response>      response);
	

private:
	std::shared_ptr<std::thread> m_threadSpin;
	std::shared_ptr<rclcpp::Node> m_node;
	std::mutex m_requestMutex;
	std::string m_name;
	IsMuseumClosingSkillCondition m_stateMachine;
	std::atomic<Status> m_tickResult{Status::undefined};
	rclcpp::Service<bt_interfaces::srv::TickCondition>::SharedPtr m_tickService;
	
	
	
};

