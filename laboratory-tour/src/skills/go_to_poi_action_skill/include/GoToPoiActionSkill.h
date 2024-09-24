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
#include "rclcpp_action/rclcpp_action.hpp"
#include "GoToPoiActionSkillSM.h"
#include <bt_interfaces/msg/action_response.hpp>
#include <scheduler_interfaces/srv/get_current_poi.hpp>
#include <blackboard_interfaces/srv/set_int_blackboard.hpp>
#include <navigation_interfaces/action/go_to_poi.hpp>
#include <bt_interfaces/srv/tick_action.hpp>
#include <bt_interfaces/srv/halt_action.hpp>

#define SERVICE_TIMEOUT 8
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
	void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
			   std::shared_ptr<bt_interfaces::srv::TickAction::Response>      response);
	void halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::HaltAction::Request> request,
			   [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::HaltAction::Response> response);

private:
	std::shared_ptr<std::thread> m_threadSpin;
	std::shared_ptr<rclcpp::Node> m_node;
	std::shared_ptr<rclcpp::Node> m_node2;
	std::mutex m_requestMutex;
	std::mutex m_actionMutex;
	std::mutex m_feedbackMutex;
	std::string m_name;
	uint m_status;
	GoToPoiActionSkillAction m_stateMachine;
	rclcpp::TimerBase::SharedPtr m_timer;
	std::atomic<Status> m_tickResult{Status::undefined};
	rclcpp::Service<bt_interfaces::srv::TickAction>::SharedPtr m_tickService;
	std::atomic<bool> m_haltResult{false};
	rclcpp::Service<bt_interfaces::srv::HaltAction>::SharedPtr m_haltService;
	std::shared_ptr<rclcpp::Node> m_nodeGoToPoi;
	rclcpp_action::Client<navigation_interfaces::action::GoToPoi>::SendGoalOptions m_send_goal_options;
	rclcpp_action::Client<navigation_interfaces::action::GoToPoi>::SharedPtr m_actionClient;
	void goal_response_callback(const  rclcpp_action::ClientGoalHandle<navigation_interfaces::action::GoToPoi>::SharedPtr & goal_handle);
	// void send_goal(std::shared_ptr<rclcpp::Node> nodeGoToPoi, rclcpp_action::Client<navigation_interfaces::action::GoToPoi>::SharedPtr clientGoToPoi);
	void send_goal(std::string poi_name);
	void feedback_callback(
    	rclcpp_action::ClientGoalHandle<navigation_interfaces::action::GoToPoi>::SharedPtr,
    	const std::shared_ptr<const navigation_interfaces::action::GoToPoi::Feedback> feedback);
	void result_callback(const  rclcpp_action::ClientGoalHandle<navigation_interfaces::action::GoToPoi>::WrappedResult & result);
};
