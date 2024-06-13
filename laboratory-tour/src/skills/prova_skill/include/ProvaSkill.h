# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "ProvaSkillSM.h"
#include <bt_interfaces/msg/condition_response.hpp>


#include <bt_interfaces/srv/tick_condition.hpp>
#include <std_msgs/msg/bool.hpp>


#define SERVICE_TIMEOUT 8

enum class Status{
	undefined,
	success,
	failure
};

class ProvaSkill
{
public:
	ProvaSkill(std::string name );
	bool start(int argc, char * argv[]);
	static void spin(std::shared_ptr<rclcpp::Node> node);
	void topic_callback(const std_msgs::msg::Bool::SharedPtr msg);
	void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickCondition::Request> request,
			   std::shared_ptr<bt_interfaces::srv::TickCondition::Response>      response);
	

private:
	std::shared_ptr<std::thread> m_threadSpin;
	std::shared_ptr<rclcpp::Node> m_node;
	std::mutex m_requestMutex;
	std::string m_name;
	ProvaSkillCondition m_stateMachine;
	std::atomic<Status> m_tickResult{Status::undefined};
	rclcpp::Service<bt_interfaces::srv::TickCondition>::SharedPtr m_tickService;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription;
	
	
};

