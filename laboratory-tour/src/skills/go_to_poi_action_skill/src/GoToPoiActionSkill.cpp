/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "GoToPoiActionSkill.h"
#include <future>
#include <QTimer>
#include <QDebug>
#include <QTime>
#include <iostream>
#include <QStateMachine>

#include <type_traits>

template<typename T>
T convert(const std::string& str) {
    if constexpr (std::is_same_v<T, int>) {
        return std::stoi(str);
    } else if constexpr (std::is_same_v<T, double>) {
        return std::stod(str);
    } else if constexpr (std::is_same_v<T, float>) {
        return std::stof(str);
    } 
    else if constexpr (std::is_same_v<T, bool>) { 
        if (str == "true" || str == "1") { 
            return true; 
        } else if (str == "false" || str == "0") { 
            return false; 
        } else { 
            throw std::invalid_argument("Invalid boolean value"); 
        } 
    } 
    else if constexpr (std::is_same_v<T, std::string>) {
        return str;
    }
    else {
        // Handle unsupported types
        throw std::invalid_argument("Unsupported type conversion");
    }
}

GoToPoiActionSkill::GoToPoiActionSkill(std::string name ) :
		m_name(std::move(name))
{
}

void GoToPoiActionSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool GoToPoiActionSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	//m_node2 = rclcpp::Node::make_shared(m_name + "Action");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "GoToPoiActionSkill::start");
	std::cout << "GoToPoiActionSkill::start";

	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickAction>(m_name + "Skill/tick",
                                                                           	std::bind(&GoToPoiActionSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));

	m_haltService = m_node->create_service<bt_interfaces_dummy::srv::HaltAction>(m_name + "Skill/halt",
                                                                            	std::bind(&GoToPoiActionSkill::halt,
                                                                            	this,
                                                                            	std::placeholders::_1,
                                                                            	std::placeholders::_2));
  
m_actionClient = rclcpp_action::create_client<navigation_interfaces_dummy::action::GoToPoi>(m_node, "/NavigationComponent/GoToPoi");
m_send_goal_options.goal_response_callback = std::bind(&GoToPoiActionSkill::goal_response_callback, this, std::placeholders::_1);
m_send_goal_options.feedback_callback =   std::bind(&GoToPoiActionSkill::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
m_send_goal_options.result_callback =  std::bind(&GoToPoiActionSkill::result_callback, this, std::placeholders::_1);

m_stateMachine.connectToEvent("SchedulerComponent.GetCurrentPoi.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeGetCurrentPoi = rclcpp::Node::make_shared(m_name + "SkillNodeGetCurrentPoi");
        std::shared_ptr<rclcpp::Client<scheduler_interfaces_dummy::srv::GetCurrentPoi>> clientGetCurrentPoi = nodeGetCurrentPoi->create_client<scheduler_interfaces_dummy::srv::GetCurrentPoi>("/SchedulerComponent/GetCurrentPoi");
        auto request = std::make_shared<scheduler_interfaces_dummy::srv::GetCurrentPoi::Request>();
        bool wait_succeded{true};
        while (!clientGetCurrentPoi->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GetCurrentPoi'. Exiting.");
                wait_succeded = false;
                m_stateMachine.submitEvent("SchedulerComponent.GetCurrentPoi.Return");
            } 
        }
        if (wait_succeded) {
            // send the request                                                                    
            auto result = clientGetCurrentPoi->async_send_request(request);
            auto futureResult = rclcpp::spin_until_future_complete(nodeGetCurrentPoi, result);
            auto response = result.get();
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
                if( response->is_ok ==true) {
                    QVariantMap data;
                    data.insert("is_ok", true);
                    data.insert("poi_number", response->poi_number.c_str());
                    m_stateMachine.submitEvent("SchedulerComponent.GetCurrentPoi.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.GetCurrentPoi.Return");
                } else {
                    QVariantMap data;
                    data.insert("is_ok", false);
                    m_stateMachine.submitEvent("SchedulerComponent.GetCurrentPoi.Return", data);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.GetCurrentPoi.Return");
                }
            }
        }
    });

m_stateMachine.connectToEvent("NavigationComponent.GoToPoi.SendGoal", [this]([[maybe_unused]]const QScxmlEvent & event){
      RCLCPP_INFO(m_node->get_logger(), "GoToPoiActionSkill::NavigationComponent.GoToPoi.SendGoal");
      RCLCPP_INFO(m_node->get_logger(), "calling send goal");
      // m_actionClient->async_cancel_all_goals(); //No longer necessary since the action server is now preemptable
      std::shared_ptr<rclcpp::Node> nodeGoToPoi = rclcpp::Node::make_shared(m_name + "SkillNodeGoToPoi");
      rclcpp_action::Client<navigation_interfaces_dummy::action::GoToPoi>::SharedPtr clientGoToPoi  =
      rclcpp_action::create_client<navigation_interfaces_dummy::action::GoToPoi>(nodeGoToPoi, "/NavigationComponent/GoToPoi");
      int poi_number = event.data().toMap()["poi_number"];
      // auto goal = std::make_shared<navigation_interfaces_dummy::action::GoToPoi::Goal>();
      // bool wait_succeded{true};
      // int retries = 0;
      RCLCPP_INFO(m_node->get_logger(), "calling send goal");
      send_goal(poi_number);
      RCLCPP_INFO(m_node->get_logger(), "done send goal");
  });

m_stateMachine.connectToEvent("NavigationComponent.GoToPoi.ResultRequest", [this]([[maybe_unused]]const QScxmlEvent & event){
      RCLCPP_INFO(m_node->get_logger(), "GoToPoiActionSkill::NavigationComponent.GoToPoi.ResultRequest");
      std::shared_ptr<rclcpp::Node> nodeGoToPoi = rclcpp::Node::make_shared(m_name + "SkillNodeGoToPoi");
      rclcpp_action::Client<navigation_interfaces_dummy::action::GoToPoi>::SharedPtr clientGoToPoi  =
      rclcpp_action::create_client<navigation_interfaces_dummy::action::GoToPoi>(nodeGoToPoi, "/NavigationComponent/GoToPoi");
      // auto goal = std::make_shared<navigation_interfaces_dummy::action::GoToPoi::Goal>();
      // bool wait_succeded{true};
      // int retries = 0;
      RCLCPP_INFO(m_node->get_logger(), "result request");
      //send_goal();
  });

m_stateMachine.connectToEvent("NavigationComponent.GoToPoi.Feedback", [this]([[maybe_unused]]const QScxmlEvent & event){
  RCLCPP_INFO(m_node->get_logger(), "NavigationComponent.GoToPoi.Feedback");
    QVariantMap data;
    m_feedbackMutex.lock();
    data.insert("status", m_status);
    m_feedbackMutex.unlock();
    m_stateMachine.submitEvent("NavigationComponent.GoToPoi.FeedbackReturn", data);
    RCLCPP_INFO(m_node->get_logger(), "NavigationComponent.GoToPoi.FeedbackReturn");
  });

	m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
		RCLCPP_INFO(m_node->get_logger(), "GoToPoiActionSkill::tickReturn %s", event.data().toMap()["status"].toString().toStdString().c_str());
		std::string result = event.data().toMap()["status"].toString().toStdString();
		if (result == std::to_string(SKILL_SUCCESS) )
		{
			m_tickResult.store(Status::success);
		}
		else if (result == std::to_string(SKILL_RUNNING) )
		{
			m_tickResult.store(Status::running);
		}
		else if (result == std::to_string(SKILL_FAILURE) )
		{ 
			m_tickResult.store(Status::failure);
		}
	});


  m_stateMachine.connectToEvent("BlackboardComponent.SetInt.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeSetInt = rclcpp::Node::make_shared(m_name + "SkillNodeSetInt");
      std::shared_ptr<rclcpp::Client<blackboard_interfaces_dummy::srv::SetIntBlackboard>> clientSetInt = nodeSetInt->create_client<blackboard_interfaces_dummy::srv::SetIntBlackboard>("/BlackboardComponent/SetInt");
      auto request = std::make_shared<blackboard_interfaces_dummy::srv::SetIntBlackboard::Request>();
      auto eventParams = event.data().toMap();
      request->field_name = convert<decltype(request->field_name)>(eventParams["field_name"].toString().toStdString());
      request->value = convert<decltype(request->value)>(eventParams["value"].toString().toStdString());
      bool wait_succeded{true};
      while (!clientSetInt->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'SetInt'. Exiting.");
              wait_succeded = false;
              m_stateMachine.submitEvent("BlackboardComponent.SetInt.Return");
          } 
      }
      if (wait_succeded) {
          // send the request                                                                    
          auto result = clientSetInt->async_send_request(request);
          auto futureResult = rclcpp::spin_until_future_complete(nodeSetInt, result);
          auto response = result.get();
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              if( response->is_ok ==true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  m_stateMachine.submitEvent("BlackboardComponent.SetInt.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BlackboardComponent.SetInt.Return");
              } else {
                  QVariantMap data;
                  data.insert("is_ok", false);
                  m_stateMachine.submitEvent("BlackboardComponent.SetInt.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BlackboardComponent.SetInt.Return");
              }
          }
      }
  });

	m_stateMachine.connectToEvent("HALT_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
    m_actionClient->async_cancel_all_goals();
		RCLCPP_INFO(m_node->get_logger(), "GoToPoiActionSkill::haltresponse");
		m_haltResult.store(true);
	});
	m_stateMachine.start();
  //send_goal();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void GoToPoiActionSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "GoToPoiActionSkill::tick");
    auto message = bt_interfaces_dummy::msg::ActionResponse();
    m_tickResult.store(Status::undefined); //here we can put a struct
    m_stateMachine.submitEvent("CMD_TICK");
   
    while(m_tickResult.load()== Status::undefined) 
    {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
        // qInfo() <<  "active names" << m_stateMachine.activeStateNames();
    }
    switch(m_tickResult.load()) 
    {
        case Status::running:
            response->status = SKILL_RUNNING;
            break;
        case Status::failure:
            response->status = SKILL_FAILURE;
            break;
        case Status::success:
            response->status = SKILL_SUCCESS;
            break;            
    }
    RCLCPP_INFO(m_node->get_logger(), "GoToPoiActionSkill::tickDone");
   
    response->is_ok = true;
}

void GoToPoiActionSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Request> request,
    [[maybe_unused]] std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Response> response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "GoToPoiActionSkill::halt");
    m_haltResult.store(false); //here we can put a struct
    m_stateMachine.submitEvent("CMD_HALT");
   
    while(!m_haltResult.load()) 
    {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
        // qInfo() <<  "active names" << m_stateMachine.activeStateNames();
    }
    RCLCPP_INFO(m_node->get_logger(), "GoToPoiActionSkill::haltDone");
   
    response->is_ok = true;
}

void GoToPoiActionSkill::send_goal(int poi_number)
  {
    using namespace std::placeholders;
    bool wait_succeded{true};
    int retries = 0;

    while (!m_actionClient->wait_for_action_server(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GoToPoi'. Exiting.");
        wait_succeded = false;
        break;
      } 
      retries++;
      if(retries == SERVICE_TIMEOUT) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'GoToPoi'.");
        wait_succeded = false;
        QVariantMap data;
        data.insert("is_ok", false);
        m_stateMachine.submitEvent("NavigationComponent.GoToPoi.GoalResponse", data);
        break;
      }
    }
  if (wait_succeded) {
      auto goal_msg = navigation_interfaces_dummy::action::GoToPoi::Goal();
      RCLCPP_INFO(m_node->get_logger(), "Sending goal");
      goal_msg.poi_number = poi_number;
      m_actionClient->async_send_goal(goal_msg, m_send_goal_options);
      QVariantMap data;
      data.insert("is_ok", true);
      m_stateMachine.submitEvent("NavigationComponent.GoToPoi.GoalResponse", data);
    }
  }

void GoToPoiActionSkill::goal_response_callback(const rclcpp_action::ClientGoalHandle<navigation_interfaces_dummy::action::GoToPoi>::SharedPtr & goal_handle)
{
  std::cout << "Provaa" << std::endl;
  QVariantMap data;
  if (!goal_handle) {
    data.insert("is_ok", false);
    m_stateMachine.submitEvent("NavigationComponent.GoToPoi.GoalResponse", data);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NavigationComponent.GoToPoi.GoalResponse Failure");
    RCLCPP_ERROR(m_node->get_logger(), "Goal was rejected by server");
  } else {
    data.insert("is_ok", true);
    m_stateMachine.submitEvent("NavigationComponent.GoToPoi.GoalResponse", data);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NavigationComponent.GoToPoi.GoalResponse Success");
    RCLCPP_INFO(m_node->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void GoToPoiActionSkill::feedback_callback(
    rclcpp_action::ClientGoalHandle<navigation_interfaces_dummy::action::GoToPoi>::SharedPtr,
  const std::shared_ptr<const navigation_interfaces_dummy::action::GoToPoi::Feedback> feedback)
{
  m_status = feedback->status.status;
  std::cout << "Status " << m_status << std::endl;
}

void GoToPoiActionSkill::result_callback(const  rclcpp_action::ClientGoalHandle<navigation_interfaces_dummy::action::GoToPoi>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(m_node->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(m_node->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(m_node->get_logger(), "Unknown result code");
      break;
  }
  //std::cout << "Result received: " << result.result->is_ok << std::endl;
  RCLCPP_INFO(m_node->get_logger(), "Result received: %d ", result.result->is_ok);
  QVariantMap data;
  data.insert("is_ok", result.result->is_ok);
  m_stateMachine.submitEvent("NavigationComponent.GoToPoi.ResultResponse", data);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NavigationComponent.GoToPoi.ResultResponse");
}