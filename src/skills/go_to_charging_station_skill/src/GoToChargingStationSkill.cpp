#include "GoToChargingStationSkill.h"
#include <future>
#include <QTimer>
#include <QDebug>
#include <QCoreApplication>

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

GoToChargingStationSkill::GoToChargingStationSkill(std::string name ) :
		m_name(std::move(name))
{
    
}

GoToChargingStationSkill::~GoToChargingStationSkill()
{
    //std::cout << "DEBUG: Invoked destructor of GoToChargingStationSkill" << std::endl;
    m_threadSpin->join();
}

void GoToChargingStationSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::spin(node);
    rclcpp::shutdown();
    QCoreApplication::quit();
    //std::cout << "DEBUG: GoToChargingStationSkill::spin successfully ended" << std::endl;
}

bool GoToChargingStationSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "GoToChargingStationSkill::start");
	std::cout << "DEBUG: GoToChargingStationSkill::start" << std::endl;

  
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickAction>(m_name + "Skill/tick",
                                                                           	std::bind(&GoToChargingStationSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
  
	m_haltService = m_node->create_service<bt_interfaces_dummy::srv::HaltAction>(m_name + "Skill/halt",
                                                                            	std::bind(&GoToChargingStationSkill::halt,
                                                                            	this,
                                                                            	std::placeholders::_1,
                                                                            	std::placeholders::_2));
  
  m_actionClient = rclcpp_action::create_client<navigation_interfaces::action::GoToPoi>(m_node, "/NavigationComponent/GoToPoi");
  m_send_goal_options.goal_response_callback = std::bind(&GoToChargingStationSkill::goal_response_callback, this, std::placeholders::_1);
  m_send_goal_options.feedback_callback =   std::bind(&GoToChargingStationSkill::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  m_send_goal_options.result_callback =  std::bind(&GoToChargingStationSkill::result_callback, this, std::placeholders::_1);
  
  
  
  
  m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "GoToChargingStationSkill::tickReturn %s", event.data().toMap()["status"].toString().toStdString().c_str());
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
    
  m_stateMachine.connectToEvent("HALT_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "GoToChargingStationSkill::haltresponse");
    m_haltResult.store(true);
  });

  m_stateMachine.connectToEvent("NavigationComponent.GoToPoi.SendGoal", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "GoToChargingStationSkill::NavigationComponent.GoToPoi.SendGoal");
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
      RCLCPP_INFO(m_node->get_logger(), "GoToChargingStationSkill::NavigationComponent.GoToPoi.ResultRequest");
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

void GoToChargingStationSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Response>      response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "GoToChargingStationSkill::tick");
  m_tickResult.store(Status::undefined);
  m_stateMachine.submitEvent("CMD_TICK");
  
  while(m_tickResult.load()== Status::undefined) {
      std::this_thread::sleep_for (std::chrono::milliseconds(100));
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
      case Status::undefined:
          response->status = SKILL_FAILURE;
          break;
  }
  RCLCPP_INFO(m_node->get_logger(), "GoToChargingStationSkill::tickDone");
  response->is_ok = true;
}

void GoToChargingStationSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Request> request,
    [[maybe_unused]] std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Response> response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "GoToChargingStationSkill::halt");
  m_haltResult.store(false);
  m_stateMachine.submitEvent("CMD_HALT");
  while(!m_haltResult.load()) {
      std::this_thread::sleep_for (std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(m_node->get_logger(), "GoToChargingStationSkill::haltDone");
  response->is_ok = true;
}






void GoToChargingStationSkill::send_goal(navigation_interfaces::action::GoToPoi::Goal goal_msg)
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

void GoToChargingStationSkill::result_callback(const  rclcpp_action::ClientGoalHandle<navigation_interfaces::action::GoToPoi>::WrappedResult & result)
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
void GoToChargingStationSkill::goal_response_callback(const rclcpp_action::ClientGoalHandle<navigation_interfaces::action::GoToPoi>::SharedPtr & goal_handle)
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

void GoToChargingStationSkill::feedback_callback(
    rclcpp_action::ClientGoalHandle<navigation_interfaces::action::GoToPoi>::SharedPtr,
  const std::shared_ptr<const navigation_interfaces::action::GoToPoi::Feedback> feedback)
{
  
}



