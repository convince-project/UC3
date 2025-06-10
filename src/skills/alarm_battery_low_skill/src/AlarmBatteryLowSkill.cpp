#include "AlarmBatteryLowSkill.h"
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

AlarmBatteryLowSkill::AlarmBatteryLowSkill(std::string name ) :
		m_name(std::move(name))
{
    
}

void AlarmBatteryLowSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool AlarmBatteryLowSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "AlarmBatteryLowSkill::start");
	std::cout << "AlarmBatteryLowSkill::start";

  
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickCondition>(m_name + "Skill/tick",
                                                                           	std::bind(&AlarmBatteryLowSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
  
  
  
  
  m_stateMachine.connectToEvent("NotifyUserComponent.StartAlarm.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeStartAlarm = rclcpp::Node::make_shared(m_name + "SkillNodeStartAlarm");
      std::shared_ptr<rclcpp::Client<notify_user_interfaces::srv::StartAlarm>> clientStartAlarm = nodeStartAlarm->create_client<notify_user_interfaces::srv::StartAlarm>("/NotifyUserComponent/StartAlarm");
      auto request = std::make_shared<notify_user_interfaces::srv::StartAlarm::Request>();
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
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  data.insert("is_ok", response->is_ok);
                  m_stateMachine.submitEvent("NotifyUserComponent.StartAlarm.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NotifyUserComponent.StartAlarm.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'StartAlarm'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
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
    else if (result == std::to_string(SKILL_FAILURE) )
    { 
      m_tickResult.store(Status::failure);
    }
  });
    

  
  
  
  

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void AlarmBatteryLowSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickCondition::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickCondition::Response>      response)
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








