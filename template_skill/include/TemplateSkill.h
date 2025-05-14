# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "$className$SM.h"
#include <bt_interfaces_dummy/msg/$skillTypeLC$_response.hpp>/*INTERFACES_LIST*/
/*INTERFACE*/
#include <$eventData.interfaceName$/srv/$eventData.functionNameSnakeCase$.hpp> /*END_INTERFACE*/
/*ACTION_INTERFACE*/
#include <$eventData.interfaceName$/action/$eventData.functionNameSnakeCase$.hpp> /*END_ACTION_INTERFACE*/
/*TOPIC_INTERFACE*/
#include <$eventData.interfaceData[interfaceDataType]$.hpp> /*END_TOPIC_INTERFACE*/
/*TICK*/#include <bt_interfaces_dummy/srv/tick_$skillTypeLC$.hpp>/*END_TICK*/
/*HALT*/#include <bt_interfaces_dummy/srv/halt_$skillTypeLC$.hpp>/*END_HALT*/
/*DATAMODEL*/
#include "$skillName$SkillDataModel.h" /*END_DATAMODEL*/

#define SERVICE_TIMEOUT 8
#define SKILL_SUCCESS 0
#define SKILL_FAILURE 1
#define SKILL_RUNNING 2

enum class Status{
	undefined,/*ACTION*/
	running, /*END_ACTION*/
	success,
	failure
};

class $className$
{
public:
	$className$(std::string name );
	bool start(int argc, char * argv[]);
	static void spin(std::shared_ptr<rclcpp::Node> node);
	/*TICK_CMD*/
	void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::Tick$skillType$::Request> request,
			   std::shared_ptr<bt_interfaces_dummy::srv::Tick$skillType$::Response>      response);/*END_TICK_CMD*/
	/*HALT_CMD*/
	void halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Request> request,
			   [[maybe_unused]] std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Response> response);/*END_HALT_CMD*/
	/*TOPIC_CALLBACK_LIST_H*/
	/*TOPIC_CALLBACK_H*/void topic_callback_$eventData.functionName$(const $eventData.interfaceData[interfaceDataType]$::SharedPtr msg);/*END_TOPIC_CALLBACK_H*/

private:
	std::shared_ptr<std::thread> m_threadSpin;
	std::shared_ptr<rclcpp::Node> m_node;
	std::mutex m_requestMutex;
	std::string m_name;
	$SMName$ m_stateMachine;
	/*TICK_RESPONSE*/std::atomic<Status> m_tickResult{Status::undefined};/*END_TICK_RESPONSE*/
	/*TICK_CMD*/rclcpp::Service<bt_interfaces_dummy::srv::Tick$skillType$>::SharedPtr m_tickService;/*END_TICK_CMD*/
	/*HALT_RESPONSE*/std::atomic<bool> m_haltResult{false};/*END_HALT_RESPONSE*/
	/*HALT_CMD*/rclcpp::Service<bt_interfaces_dummy::srv::HaltAction>::SharedPtr m_haltService;/*END_HALT_CMD*/
	/*DATAMODEL*/$skillName$SkillDataModel m_dataModel; /*END_DATAMODEL*/
	/*TOPIC_SUBSCRIPTIONS_LIST_H*/
	/*TOPIC_SUBSCRIPTION_H*/
	rclcpp::Subscription<$eventData.interfaceData[interfaceDataType]$>::SharedPtr m_subscription_$eventData.functionName$;/*END_TOPIC_SUBSCRIPTION_H*/
	/*ACTION_LIST_H*//*ACTION_H*/
	std::shared_ptr<rclcpp::Node> m_node_action;
	std::mutex m_actionMutex;
	std::mutex m_feedbackMutex;
	rclcpp_action::Client<$eventData.interfaceName$::action::$eventData.functionName$>::SendGoalOptions m_send_goal_options;
	rclcpp_action::Client<$eventData.interfaceName$::action::$eventData.functionName$>::SharedPtr m_actionClient;
	void goal_response_callback(const  rclcpp_action::ClientGoalHandle<$eventData.interfaceName$::action::$eventData.functionName$>::SharedPtr & goal_handle);
	void send_goal($eventData.interfaceName$::action::$eventData.functionName$::Goal);
	void feedback_callback(
    	rclcpp_action::ClientGoalHandle<$eventData.interfaceName$::action::$eventData.functionName$>::SharedPtr,
    	const std::shared_ptr<const $eventData.interfaceName$::action::$eventData.functionName$::Feedback> feedback);
	void result_callback(const  rclcpp_action::ClientGoalHandle<$eventData.interfaceName$::action::$eventData.functionName$>::WrappedResult & result);
	/*END_ACTION_H*/
	/*FEEDBACK_DATA_LIST*//*FEEDBACK_DATA*/
  	$eventData.interfaceDataType$ m_$eventData.interfaceDataField$;
  	/*END_FEEDBACK_DATA*/

};

