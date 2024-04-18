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
#include "VisitorsFollowingRobotSkillSM.h"
#include <bt_interfaces/msg/condition_response.hpp>
#include "VisitorsFollowingRobotSkillDataModel.h"
#include <bt_interfaces/srv/tick_condition.hpp>

enum class Status{
	undefined,
	success,
	failure
};

class VisitorsFollowingRobotSkill
{
public:
	VisitorsFollowingRobotSkill(std::string name );
	bool start(int argc, char * argv[]);
	static void spin(std::shared_ptr<rclcpp::Node> node);
	void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickCondition::Request> request,
			   std::shared_ptr<bt_interfaces::srv::TickCondition::Response>      response);

private:
	std::shared_ptr<std::thread> m_threadSpin;
	std::shared_ptr<rclcpp::Node> m_node;
	std::mutex m_requestMutex;
	std::string m_name;
	VisitorsFollowingRobotSkillCondition m_stateMachine;
	
VisitorsFollowingRobotSkillDataModel m_dataModel;
	std::atomic<Status> m_tickResult{Status::undefined};
	rclcpp::Service<bt_interfaces::srv::TickCondition>::SharedPtr m_tickService;
	
};

