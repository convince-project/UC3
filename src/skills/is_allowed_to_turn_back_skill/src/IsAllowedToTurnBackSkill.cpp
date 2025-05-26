#include "IsAllowedToTurnBackSkill.h"
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

IsAllowedToTurnBackSkill::IsAllowedToTurnBackSkill(std::string name ) :
		m_name(std::move(name))
{
    
}

void IsAllowedToTurnBackSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool IsAllowedToTurnBackSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "IsAllowedToTurnBackSkill::start");
	std::cout << "IsAllowedToTurnBackSkill::start";

  
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickCondition>(m_name + "Skill/tick",
                                                                           	std::bind(&IsAllowedToTurnBackSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
  
  
  
  
  m_stateMachine.connectToEvent("TurnBackManagerComponent.IsAllowedToTurnBack.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeIsAllowedToTurnBack = rclcpp::Node::make_shared(m_name + "SkillNodeIsAllowedToTurnBack");
      std::shared_ptr<rclcpp::Client<turn_back_manager_interfaces::srv::IsAllowedToTurnBack>> clientIsAllowedToTurnBack = nodeIsAllowedToTurnBack->create_client<turn_back_manager_interfaces::srv::IsAllowedToTurnBack>("/TurnBackManagerComponent/IsAllowedToTurnBack");
      auto request = std::make_shared<turn_back_manager_interfaces::srv::IsAllowedToTurnBack::Request>();
      auto eventParams = event.data().toMap();
      
      bool wait_succeded{true};
      int retries = 0;
      while (!clientIsAllowedToTurnBack->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'IsAllowedToTurnBack'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'IsAllowedToTurnBack'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientIsAllowedToTurnBack->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeIsAllowedToTurnBack, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  data.insert("is_allowed", response->is_allowed);
                  m_stateMachine.submitEvent("TurnBackManagerComponent.IsAllowedToTurnBack.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TurnBackManagerComponent.IsAllowedToTurnBack.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'IsAllowedToTurnBack'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("TurnBackManagerComponent.IsAllowedToTurnBack.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TurnBackManagerComponent.IsAllowedToTurnBack.Return");
  });
  m_stateMachine.connectToEvent("BlackboardComponent.GetString.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeGetString = rclcpp::Node::make_shared(m_name + "SkillNodeGetString");
      std::shared_ptr<rclcpp::Client<blackboard_interfaces::srv::GetStringBlackboard>> clientGetString = nodeGetString->create_client<blackboard_interfaces::srv::GetStringBlackboard>("/BlackboardComponent/GetString");
      auto request = std::make_shared<blackboard_interfaces::srv::GetStringBlackboard::Request>();
      auto eventParams = event.data().toMap();
      
      request->field_name = convert<decltype(request->field_name)>(eventParams["field_name"].toString().toStdString());
      bool wait_succeded{true};
      int retries = 0;
      while (!clientGetString->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GetString'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'GetString'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientGetString->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeGetString, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  data.insert("value", response->value.c_str());
                  m_stateMachine.submitEvent("BlackboardComponent.GetString.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BlackboardComponent.GetString.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'GetString'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("BlackboardComponent.GetString.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BlackboardComponent.GetString.Return");
  });
  
  m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "IsAllowedToTurnBackSkill::tickReturn %s", event.data().toMap()["status"].toString().toStdString().c_str());
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
    

  
  
  
  

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void IsAllowedToTurnBackSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickCondition::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickCondition::Response>      response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "IsAllowedToTurnBackSkill::tick");
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
      case Status::undefined:   
          response->status = SKILL_FAILURE; // Default case, should not happen
          break;          
  }
  RCLCPP_INFO(m_node->get_logger(), "IsAllowedToTurnBackSkill::tickDone");
  response->is_ok = true;
}








