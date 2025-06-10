# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "GoToPoiActionSkillSM.h"
#include <bt_interfaces_dummy/msg/action_response.hpp>
#include <scheduler_interfaces/srv/get_current_poi.hpp> 
#include <navigation_interfaces/action/go_to_poi.hpp> 



#include <bt_interfaces_dummy/srv/tick_action.hpp>
#include <bt_interfaces_dummy/srv/halt_action.hpp>


#define SERVICE_TIMEOUT 8
#define SKILL_SUCCESS 0
#define SKILL_FAILURE 1
#define SKILL_RUNNING 2

enum class Status{
	undefined,
	running, 
	success,
	failure
};

class GoToPoiActionSkill
{
public:
	GoToPoiActionSkill(std::string name );
	bool start(int argc, char * argv[]);
	static void spin(std::shared_ptr<rclcpp::Node> node);
	
	void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Request> request,
			   std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Response>      response);
	
	void halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Request> request,
			   [[maybe_unused]] std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Response> response);
	
	

private:
	std::shared_ptr<std::thread> m_threadSpin;
	std::shared_ptr<rclcpp::Node> m_node;
	std::mutex m_requestMutex;
	std::string m_name;
	GoToPoiActionSkillAction m_stateMachine;
	std::atomic<Status> m_tickResult{Status::undefined};
	rclcpp::Service<bt_interfaces_dummy::srv::TickAction>::SharedPtr m_tickService;
	std::atomic<bool> m_haltResult{false};
	rclcpp::Service<bt_interfaces_dummy::srv::HaltAction>::SharedPtr m_haltService;
	
	
	
	
	std::shared_ptr<rclcpp::Node> m_node_action;
	std::mutex m_actionMutex;
	std::mutex m_feedbackMutex;
	rclcpp_action::Client<navigation_interfaces::action::GoToPoi>::SendGoalOptions m_send_goal_options;
	rclcpp_action::Client<navigation_interfaces::action::GoToPoi>::SharedPtr m_actionClient;
	void goal_response_callback(const  rclcpp_action::ClientGoalHandle<navigation_interfaces::action::GoToPoi>::SharedPtr & goal_handle);
	void send_goal(navigation_interfaces::action::GoToPoi::Goal);
	void feedback_callback(
    	rclcpp_action::ClientGoalHandle<navigation_interfaces::action::GoToPoi>::SharedPtr,
    	const std::shared_ptr<const navigation_interfaces::action::GoToPoi::Feedback> feedback);
	void result_callback(const  rclcpp_action::ClientGoalHandle<navigation_interfaces::action::GoToPoi>::WrappedResult & result);
	
	

};

