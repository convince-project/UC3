#include "AlarmBatteryLowSkill.h"
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

AlarmBatteryLowSkill::AlarmBatteryLowSkill(std::string name ) :
		m_name(std::move(name))
{
    
}

AlarmBatteryLowSkill::~AlarmBatteryLowSkill()
{
    //std::cout << "DEBUG: Invoked destructor of AlarmBatteryLowSkill" << std::endl;
    m_threadSpin->join();
}

void AlarmBatteryLowSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::spin(node);
    rclcpp::shutdown();
    QCoreApplication::quit();
    //std::cout << "DEBUG: AlarmBatteryLowSkill::spin successfully ended" << std::endl;
}

bool AlarmBatteryLowSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "AlarmBatteryLowSkill::start");
	std::cout << "DEBUG: AlarmBatteryLowSkill::start" << std::endl;

  
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickAction>(m_name + "Skill/tick",
                                                                           	std::bind(&AlarmBatteryLowSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
  m_tickService->configure_introspection(m_node->get_clock(), rclcpp::SystemDefaultsQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);
  
	m_haltService = m_node->create_service<bt_interfaces_dummy::srv::HaltAction>(m_name + "Skill/halt",
                                                                            	std::bind(&AlarmBatteryLowSkill::halt,
                                                                            	this,
                                                                            	std::placeholders::_1,
                                                                            	std::placeholders::_2));
  m_haltService->configure_introspection(m_node->get_clock(), rclcpp::SystemDefaultsQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);
  
  
  
  m_stateMachine.connectToEvent("NotifyUserComponent.StopAlarm.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeStopAlarm = rclcpp::Node::make_shared(m_name + "SkillNodeStopAlarm");
      std::shared_ptr<rclcpp::Client<notify_user_interfaces::srv::StopAlarm>> clientStopAlarm = nodeStopAlarm->create_client<notify_user_interfaces::srv::StopAlarm>("/NotifyUserComponent/StopAlarm");
      auto request = std::make_shared<notify_user_interfaces::srv::StopAlarm::Request>();
      clientStopAlarm->configure_introspection(nodeStopAlarm->get_clock(), rclcpp::SystemDefaultsQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);
      auto eventParams = event.data().toMap();
      
      bool wait_succeded{true};
      int retries = 0;
      while (!clientStopAlarm->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'StopAlarm'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'StopAlarm'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientStopAlarm->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeStopAlarm, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              QVariantMap data;
              data.insert("call_succeeded", true);
              m_stateMachine.submitEvent("NotifyUserComponent.StopAlarm.Return", data);
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NotifyUserComponent.StopAlarm.Return");
              return;
              
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'StopAlarm'.");
          }
      }
      QVariantMap data;
      data.insert("call_succeeded", false);
      m_stateMachine.submitEvent("NotifyUserComponent.StopAlarm.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NotifyUserComponent.StopAlarm.Return");
  });
  m_stateMachine.connectToEvent("NotifyUserComponent.StartAlarm.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeStartAlarm = rclcpp::Node::make_shared(m_name + "SkillNodeStartAlarm");
      std::shared_ptr<rclcpp::Client<notify_user_interfaces::srv::StartAlarm>> clientStartAlarm = nodeStartAlarm->create_client<notify_user_interfaces::srv::StartAlarm>("/NotifyUserComponent/StartAlarm");
      auto request = std::make_shared<notify_user_interfaces::srv::StartAlarm::Request>();
      clientStartAlarm->configure_introspection(nodeStartAlarm->get_clock(), rclcpp::SystemDefaultsQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);
      auto eventParams = event.data().toMap();
      
      bool wait_succeded{true};
      int retries = 0;
      while (!clientStartAlarm->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'StartAlarm'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'StartAlarm'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientStartAlarm->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeStartAlarm, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              QVariantMap data;
              data.insert("call_succeeded", true);
              m_stateMachine.submitEvent("NotifyUserComponent.StartAlarm.Return", data);
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NotifyUserComponent.StartAlarm.Return");
              return;
              
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'StartAlarm'.");
          }
      }
      QVariantMap data;
      data.insert("call_succeeded", false);
      m_stateMachine.submitEvent("NotifyUserComponent.StartAlarm.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NotifyUserComponent.StartAlarm.Return");
  });
  
  m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "AlarmBatteryLowSkill::tickReturn %s", event.data().toMap()["status"].toString().toStdString().c_str());
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
    RCLCPP_INFO(m_node->get_logger(), "AlarmBatteryLowSkill::haltresponse");
    m_haltResult.store(true);
  });

  
  
  
  

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);
       
	return true;
}

void AlarmBatteryLowSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Response>      response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "AlarmBatteryLowSkill::tick");
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
  RCLCPP_INFO(m_node->get_logger(), "AlarmBatteryLowSkill::tickDone");
  response->is_ok = true;
}

void AlarmBatteryLowSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Request> request,
    [[maybe_unused]] std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Response> response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "AlarmBatteryLowSkill::halt");
  m_haltResult.store(false);
  m_stateMachine.submitEvent("CMD_HALT");
  while(!m_haltResult.load()) {
      std::this_thread::sleep_for (std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(m_node->get_logger(), "AlarmBatteryLowSkill::haltDone");
  response->is_ok = true;
}








