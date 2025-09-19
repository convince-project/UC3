#include "StartPoiTimerSkill.h"
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

StartPoiTimerSkill::StartPoiTimerSkill(std::string name ) :
		m_name(std::move(name))
{
    
}

StartPoiTimerSkill::~StartPoiTimerSkill()
{
    //std::cout << "DEBUG: Invoked destructor of StartPoiTimerSkill" << std::endl;
    m_threadSpin->join();
}

void StartPoiTimerSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::spin(node);
    rclcpp::shutdown();
    QCoreApplication::quit();
    //std::cout << "DEBUG: StartPoiTimerSkill::spin successfully ended" << std::endl;
}

bool StartPoiTimerSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "StartPoiTimerSkill::start");
	std::cout << "DEBUG: StartPoiTimerSkill::start" << std::endl;

  
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickAction>(m_name + "Skill/tick",
                                                                           	std::bind(&StartPoiTimerSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
  
	m_haltService = m_node->create_service<bt_interfaces_dummy::srv::HaltAction>(m_name + "Skill/halt",
                                                                            	std::bind(&StartPoiTimerSkill::halt,
                                                                            	this,
                                                                            	std::placeholders::_1,
                                                                            	std::placeholders::_2));
  
  
  
  m_stateMachine.connectToEvent("TimeComponent.StartPoiTimer.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeStartPoiTimer = rclcpp::Node::make_shared(m_name + "SkillNodeStartPoiTimer");
      std::shared_ptr<rclcpp::Client<time_interfaces::srv::StartPoiTimer>> clientStartPoiTimer = nodeStartPoiTimer->create_client<time_interfaces::srv::StartPoiTimer>("/TimeComponent/StartPoiTimer");
      auto request = std::make_shared<time_interfaces::srv::StartPoiTimer::Request>();
      auto eventParams = event.data().toMap();
      
      bool wait_succeded{true};
      int retries = 0;
      while (!clientStartPoiTimer->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'StartPoiTimer'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'StartPoiTimer'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientStartPoiTimer->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeStartPoiTimer, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  m_stateMachine.submitEvent("TimeComponent.StartPoiTimer.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TimeComponent.StartPoiTimer.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'StartPoiTimer'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("TimeComponent.StartPoiTimer.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TimeComponent.StartPoiTimer.Return");
  });
  m_stateMachine.connectToEvent("BlackboardComponent.SetInt.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeSetInt = rclcpp::Node::make_shared(m_name + "SkillNodeSetInt");
      std::shared_ptr<rclcpp::Client<blackboard_interfaces::srv::SetIntBlackboard>> clientSetInt = nodeSetInt->create_client<blackboard_interfaces::srv::SetIntBlackboard>("/BlackboardComponent/SetInt");
      auto request = std::make_shared<blackboard_interfaces::srv::SetIntBlackboard::Request>();
      auto eventParams = event.data().toMap();
      
      request->value = convert<decltype(request->value)>(eventParams["value"].toString().toStdString());
      request->field_name = convert<decltype(request->field_name)>(eventParams["field_name"].toString().toStdString());
      bool wait_succeded{true};
      int retries = 0;
      while (!clientSetInt->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'SetInt'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'SetInt'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientSetInt->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeSetInt, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  m_stateMachine.submitEvent("BlackboardComponent.SetInt.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BlackboardComponent.SetInt.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'SetInt'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("BlackboardComponent.SetInt.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BlackboardComponent.SetInt.Return");
  });
  
  m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "StartPoiTimerSkill::tickReturn %s", event.data().toMap()["status"].toString().toStdString().c_str());
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
    RCLCPP_INFO(m_node->get_logger(), "StartPoiTimerSkill::haltresponse");
    m_haltResult.store(true);
  });

  
  
  
  

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);
       
	return true;
}

void StartPoiTimerSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Response>      response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "StartPoiTimerSkill::tick");
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
  RCLCPP_INFO(m_node->get_logger(), "StartPoiTimerSkill::tickDone");
  response->is_ok = true;
}

void StartPoiTimerSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Request> request,
    [[maybe_unused]] std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Response> response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "StartPoiTimerSkill::halt");
  m_haltResult.store(false);
  m_stateMachine.submitEvent("CMD_HALT");
  while(!m_haltResult.load()) {
      std::this_thread::sleep_for (std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(m_node->get_logger(), "StartPoiTimerSkill::haltDone");
  response->is_ok = true;
}








