#include "SetNavigationPositionSkill.h"
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

SetNavigationPositionSkill::SetNavigationPositionSkill(std::string name ) :
		m_name(std::move(name))
{
    
}

void SetNavigationPositionSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool SetNavigationPositionSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "SetNavigationPositionSkill::start");
	std::cout << "SetNavigationPositionSkill::start";

  
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickAction>(m_name + "Skill/tick",
                                                                           	std::bind(&SetNavigationPositionSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
  
	m_haltService = m_node->create_service<bt_interfaces_dummy::srv::HaltAction>(m_name + "Skill/halt",
                                                                            	std::bind(&SetNavigationPositionSkill::halt,
                                                                            	this,
                                                                            	std::placeholders::_1,
                                                                            	std::placeholders::_2));
  
  
  
  m_stateMachine.connectToEvent("ExecuteDanceComponent.IsDancing.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeIsDancing = rclcpp::Node::make_shared(m_name + "SkillNodeIsDancing");
      std::shared_ptr<rclcpp::Client<execute_dance_interfaces::srv::IsDancing>> clientIsDancing = nodeIsDancing->create_client<execute_dance_interfaces::srv::IsDancing>("/ExecuteDanceComponent/IsDancing");
      auto request = std::make_shared<execute_dance_interfaces::srv::IsDancing::Request>();
      auto eventParams = event.data().toMap();
      
      bool wait_succeded{true};
      int retries = 0;
      while (!clientIsDancing->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'IsDancing'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'IsDancing'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientIsDancing->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeIsDancing, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  data.insert("is_dancing", response->is_dancing);
                  m_stateMachine.submitEvent("ExecuteDanceComponent.IsDancing.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ExecuteDanceComponent.IsDancing.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'IsDancing'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("ExecuteDanceComponent.IsDancing.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ExecuteDanceComponent.IsDancing.Return");
  });
  m_stateMachine.connectToEvent("ExecuteDanceComponent.ExecuteDance.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeExecuteDance = rclcpp::Node::make_shared(m_name + "SkillNodeExecuteDance");
      std::shared_ptr<rclcpp::Client<execute_dance_interfaces::srv::ExecuteDance>> clientExecuteDance = nodeExecuteDance->create_client<execute_dance_interfaces::srv::ExecuteDance>("/ExecuteDanceComponent/ExecuteDance");
      auto request = std::make_shared<execute_dance_interfaces::srv::ExecuteDance::Request>();
      auto eventParams = event.data().toMap();
      
      request->dance_name = convert<decltype(request->dance_name)>(eventParams["dance_name"].toString().toStdString());
      bool wait_succeded{true};
      int retries = 0;
      while (!clientExecuteDance->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'ExecuteDance'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'ExecuteDance'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientExecuteDance->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeExecuteDance, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  m_stateMachine.submitEvent("ExecuteDanceComponent.ExecuteDance.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ExecuteDanceComponent.ExecuteDance.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'ExecuteDance'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("ExecuteDanceComponent.ExecuteDance.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ExecuteDanceComponent.ExecuteDance.Return");
  });
  
  m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "SetNavigationPositionSkill::tickReturn %s", event.data().toMap()["status"].toString().toStdString().c_str());
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
    RCLCPP_INFO(m_node->get_logger(), "SetNavigationPositionSkill::haltresponse");
    m_haltResult.store(true);
  });

  
  
  
  

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void SetNavigationPositionSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Response>      response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "SetNavigationPositionSkill::tick");
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
  RCLCPP_INFO(m_node->get_logger(), "SetNavigationPositionSkill::tickDone");
  response->is_ok = true;
}

void SetNavigationPositionSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Request> request,
    [[maybe_unused]] std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Response> response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "SetNavigationPositionSkill::halt");
  m_haltResult.store(false);
  m_stateMachine.submitEvent("CMD_HALT");
  while(!m_haltResult.load()) {
      std::this_thread::sleep_for (std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(m_node->get_logger(), "SetNavigationPositionSkill::haltDone");
  response->is_ok = true;
}








