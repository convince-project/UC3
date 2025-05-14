# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "GoToChargingStationSkillSM.h"
#include <bt_interfaces_dummy/msg/actionaction_response.hpp>
#include <navigation_interfaces/srv/stop_navigation.hpp> 
#include <navigation_interfaces/srv/go_to_poi_by_name.hpp> 
#include <navigation_interfaces/srv/get_navigation_status.hpp> 
#include <navigation_interfaces/srv/check_near_to_poi.hpp> 



#include <bt_interfaces_dummy/srv/tick_actionaction.hpp>
#include <bt_interfaces_dummy/srv/halt_actionaction.hpp>


#define SERVICE_TIMEOUT 8
#define SKILL_SUCCESS 0
#define SKILL_FAILURE 1
#define SKILL_RUNNING 2

enum class Status{
	undefined,
	success,
	failure
};

class GoToChargingStationSkill
{
public:
	GoToChargingStationSkill(std::string name );
	bool start(int argc, char * argv[]);
	static void spin(std::shared_ptr<rclcpp::Node> node);
	
	void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickActionAction::Request> request,
			   std::shared_ptr<bt_interfaces_dummy::srv::TickActionAction::Response>      response);
	
	void halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Request> request,
			   [[maybe_unused]] std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Response> response);
	
	

private:
	std::shared_ptr<std::thread> m_threadSpin;
	std::shared_ptr<rclcpp::Node> m_node;
	std::mutex m_requestMutex;
	std::string m_name;
	GoToChargingStationSkillActionAction m_stateMachine;
	std::atomic<Status> m_tickResult{Status::undefined};
	rclcpp::Service<bt_interfaces_dummy::srv::TickActionAction>::SharedPtr m_tickService;
	std::atomic<bool> m_haltResult{false};
	rclcpp::Service<bt_interfaces_dummy::srv::HaltAction>::SharedPtr m_haltService;
	
	
	
	
	

};

