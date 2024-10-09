# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "BatteryLevelSkillSM.h"
#include <bt_interfaces_dummy/msg/condition_response.hpp>
#include <std_msgs/msg/int32.hpp>




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

class BatteryLevelSkill
{
public:
	BatteryLevelSkill(std::string name );
	bool start(int argc, char * argv[]);
	static void spin(std::shared_ptr<rclcpp::Node> node);
	
	void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickCondition::Request> request,
			   std::shared_ptr<bt_interfaces_dummy::srv::TickCondition::Response>      response);
	
	void topic_callback_battery_status(const std_msgs::msg::Int32::SharedPtr msg);
	

private:
	std::shared_ptr<std::thread> m_threadSpin;
	std::shared_ptr<rclcpp::Node> m_node;
	std::mutex m_requestMutex;
	std::string m_name;
	BatteryLevelSkillCondition m_stateMachine;
	std::atomic<Status> m_tickResult{Status::undefined};
	rclcpp::Service<bt_interfaces_dummy::srv::TickCondition>::SharedPtr m_tickService;
	
	
	
	
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_subscription_battery_status;
	
};

