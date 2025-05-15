#include "PeopleLeftSkill.h"
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

PeopleLeftSkill::PeopleLeftSkill(std::string name ) :
		m_name(std::move(name))
{
    
}

void PeopleLeftSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool PeopleLeftSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "PeopleLeftSkill::start");
	std::cout << "PeopleLeftSkill::start";

  
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickActionAction>(m_name + "Skill/tick",
                                                                           	std::bind(&PeopleLeftSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
  
	m_haltService = m_node->create_service<bt_interfaces_dummy::srv::HaltActionAction>(m_name + "Skill/halt",
                                                                            	std::bind(&PeopleLeftSkill::halt,
                                                                            	this,
                                                                            	std::placeholders::_1,
                                                                            	std::placeholders::_2));
  
  
  
  m_stateMachine.connectToEvent("TurnBackManagerComponent.ResetCounters.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeResetCounters = rclcpp::Node::make_shared(m_name + "SkillNodeResetCounters");
      std::shared_ptr<rclcpp::Client<turn_back_manager_interfaces::srv::ResetCounters>> clientResetCounters = nodeResetCounters->create_client<turn_back_manager_interfaces::srv::ResetCounters>("/TurnBackManagerComponent/ResetCounters");
      auto request = std::make_shared<turn_back_manager_interfaces::srv::ResetCounters::Request>();
      auto eventParams = event.data().toMap();
      
      bool wait_succeded{true};
      int retries = 0;
      while (!clientResetCounters->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'ResetCounters'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'ResetCounters'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientResetCounters->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeResetCounters, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  m_stateMachine.submitEvent("TurnBackManagerComponent.ResetCounters.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TurnBackManagerComponent.ResetCounters.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'ResetCounters'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("TurnBackManagerComponent.ResetCounters.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TurnBackManagerComponent.ResetCounters.Return");
  });
  m_stateMachine.connectToEvent("TimeComponent.StopTourTimer.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeStopTourTimer = rclcpp::Node::make_shared(m_name + "SkillNodeStopTourTimer");
      std::shared_ptr<rclcpp::Client<time_interfaces::srv::StopTourTimer>> clientStopTourTimer = nodeStopTourTimer->create_client<time_interfaces::srv::StopTourTimer>("/TimeComponent/StopTourTimer");
      auto request = std::make_shared<time_interfaces::srv::StopTourTimer::Request>();
      auto eventParams = event.data().toMap();
      
      bool wait_succeded{true};
      int retries = 0;
      while (!clientStopTourTimer->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'StopTourTimer'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'StopTourTimer'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientStopTourTimer->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeStopTourTimer, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  m_stateMachine.submitEvent("TimeComponent.StopTourTimer.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TimeComponent.StopTourTimer.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'StopTourTimer'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("TimeComponent.StopTourTimer.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TimeComponent.StopTourTimer.Return");
  });
  m_stateMachine.connectToEvent("SchedulerComponent.Reset.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeReset = rclcpp::Node::make_shared(m_name + "SkillNodeReset");
      std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::Reset>> clientReset = nodeReset->create_client<scheduler_interfaces::srv::Reset>("/SchedulerComponent/Reset");
      auto request = std::make_shared<scheduler_interfaces::srv::Reset::Request>();
      auto eventParams = event.data().toMap();
      
      bool wait_succeded{true};
      int retries = 0;
      while (!clientReset->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'Reset'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'Reset'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientReset->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeReset, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  m_stateMachine.submitEvent("SchedulerComponent.Reset.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.Reset.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'Reset'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("SchedulerComponent.Reset.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.Reset.Return");
  });
  
  m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "PeopleLeftSkill::tickReturn %s", event.data().toMap()["status"].toString().toStdString().c_str());
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
    
  m_stateMachine.connectToEvent("HALT_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "PeopleLeftSkill::haltresponse");
    m_haltResult.store(true);
  });

  
  
  
  

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void PeopleLeftSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickActionAction::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickActionAction::Response>      response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "PeopleLeftSkill::tick");
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
  RCLCPP_INFO(m_node->get_logger(), "PeopleLeftSkill::tickDone");
  response->is_ok = true;
}

void PeopleLeftSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::HaltActionAction::Request> request,
    [[maybe_unused]] std::shared_ptr<bt_interfaces_dummy::srv::HaltActionAction::Response> response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "PeopleLeftSkill::halt");
  m_haltResult.store(false);
  m_stateMachine.submitEvent("CMD_HALT");
  while(!m_haltResult.load()) {
      std::this_thread::sleep_for (std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(m_node->get_logger(), "PeopleLeftSkill::haltDone");
  response->is_ok = true;
}








