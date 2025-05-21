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
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "GoToPoiActionSkill::start");
	std::cout << "GoToPoiActionSkill::start";

  
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickCondition>(m_name + "Skill/tick",
                                                                           	std::bind(&GoToPoiActionSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
  
  
  m_actionClient = rclcpp_action::create_client<navigation_interfaces::action::GoToPoi>(m_node, "/NavigationComponent/GoToPoi");
  m_send_goal_options.goal_response_callback = std::bind(&GoToPoiActionSkill::goal_response_callback, this, std::placeholders::_1);
  m_send_goal_options.feedback_callback =   std::bind(&GoToPoiActionSkill::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  m_send_goal_options.result_callback =  std::bind(&GoToPoiActionSkill::result_callback, this, std::placeholders::_1);
  
  
  
  m_stateMachine.connectToEvent("SchedulerComponent.GetCurrentPoi.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeGetCurrentPoi = rclcpp::Node::make_shared(m_name + "SkillNodeGetCurrentPoi");
      std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::GetCurrentPoi>> clientGetCurrentPoi = nodeGetCurrentPoi->create_client<scheduler_interfaces::srv::GetCurrentPoi>("/SchedulerComponent/GetCurrentPoi");
      auto request = std::make_shared<scheduler_interfaces::srv::GetCurrentPoi::Request>();
      auto eventParams = event.data().toMap();
      
      bool wait_succeded{true};
      int retries = 0;
      while (!clientGetCurrentPoi->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GetCurrentPoi'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'GetCurrentPoi'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientGetCurrentPoi->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeGetCurrentPoi, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  data.insert("poi_number", response->poi_number);
                  data.insert("poi_name", response->poi_name.c_str());
                  m_stateMachine.submitEvent("SchedulerComponent.GetCurrentPoi.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.GetCurrentPoi.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'GetCurrentPoi'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("SchedulerComponent.GetCurrentPoi.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.GetCurrentPoi.Return");
  });
  
  m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "GoToPoiActionSkill::tickReturn %s", event.data().toMap()["status"].toString().toStdString().c_str());
    std::string result = event.data().toMap()["status"].toString().toStdString();
    if (result == std::to_string(SKILL_SUCCESS) )
    {
      m_tickResult.store(Status::success);
    }
    else if (result == std::to_string(SKILL_FAILURE) )
    { 
      m_tickResult.store(Status::failure);
    }
  });
    

  m_stateMachine.connectToEvent("NavigationComponent.GoToPoi.SendGoal", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "GoToPoiActionSkill::NavigationComponent.GoToPoi.SendGoal");
    RCLCPP_INFO(m_node->get_logger(), "calling send goal");
    std::shared_ptr<rclcpp::Node> nodeGoToPoi = rclcpp::Node::make_shared(m_name + "SkillNodeGoToPoi");
    rclcpp_action::Client<navigation_interfaces::action::GoToPoi>::SharedPtr clientGoToPoi  =
    rclcpp_action::create_client<navigation_interfaces::action::GoToPoi>(nodeGoToPoi, "/NavigationComponent/GoToPoi");
    navigation_interfaces::action::GoToPoi::Goal goal_msg;
    
    std::string temp = event.data().toMap()["poi_name"].toString().toStdString();
    goal_msg.poi_name = convert<decltype(goal_msg.poi_name)>(temp);
    
    send_goal(goal_msg);
    RCLCPP_INFO(m_node->get_logger(), "done send goal");
  });
  m_stateMachine.connectToEvent("NavigationComponent.GoToPoi.ResultRequest", [this]([[maybe_unused]]const QScxmlEvent & event){
      RCLCPP_INFO(m_node->get_logger(), "GoToPoiActionSkill::NavigationComponent.GoToPoi.ResultRequest");
      std::shared_ptr<rclcpp::Node> nodeGoToPoi = rclcpp::Node::make_shared(m_name + "SkillNodeGoToPoi");
      rclcpp_action::Client<navigation_interfaces::action::GoToPoi>::SharedPtr clientGoToPoi  =
        rclcpp_action::create_client<navigation_interfaces::action::GoToPoi>(nodeGoToPoi, "/NavigationComponent/GoToPoi");
      RCLCPP_INFO(m_node->get_logger(), "result request");
  });
  m_stateMachine.connectToEvent("NavigationComponent.GoToPoi.Feedback", [this]([[maybe_unused]]const QScxmlEvent & event){
      RCLCPP_INFO(m_node->get_logger(), "NavigationComponent.GoToPoi.Feedback");
      QVariantMap data;
      m_feedbackMutex.lock();
      
      m_feedbackMutex.unlock();
      m_stateMachine.submitEvent("NavigationComponent.GoToPoi.FeedbackReturn", data);
      RCLCPP_INFO(m_node->get_logger(), "NavigationComponent.GoToPoi.FeedbackReturn");
  });
  
  
  

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void GoToPoiActionSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickCondition::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickCondition::Response>      response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "GoToPoiActionSkill::tick");
  m_tickResult.store(Status::undefined);
  m_stateMachine.submitEvent("CMD_TICK");
  
  while(m_tickResult.load()== Status::undefined) {
      std::this_thread::sleep_for (std::chrono::milliseconds(100));
  }
  switch(m_tickResult.load()) 
  {
      
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






void GoToPoiActionSkill::send_goal(navigation_interfaces::action::GoToPoi::Goal goal_msg)
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
      RCLCPP_INFO(m_node->get_logger(), "Sending goal");
      m_actionClient->async_send_goal(goal_msg, m_send_goal_options);
      QVariantMap data;
      data.insert("is_ok", true);
      m_stateMachine.submitEvent("NavigationComponent.GoToPoi.GoalResponse", data);
    }
  }

void GoToPoiActionSkill::result_callback(const  rclcpp_action::ClientGoalHandle<navigation_interfaces::action::GoToPoi>::WrappedResult & result)
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
void GoToPoiActionSkill::goal_response_callback(const rclcpp_action::ClientGoalHandle<navigation_interfaces::action::GoToPoi>::SharedPtr & goal_handle)
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
    rclcpp_action::ClientGoalHandle<navigation_interfaces::action::GoToPoi>::SharedPtr goal_handle,
    const std::shared_ptr<const navigation_interfaces::action::GoToPoi::Feedback> feedback)
{
    (void)goal_handle;
    (void)feedback;
}



