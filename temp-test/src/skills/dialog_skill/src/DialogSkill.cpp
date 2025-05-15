#include "DialogSkill.h"
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

DialogSkill::DialogSkill(std::string name ) :
		m_name(std::move(name))
{
    
}

void DialogSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool DialogSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "DialogSkill::start");
	std::cout << "DialogSkill::start";

  
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickAction>(m_name + "Skill/tick",
                                                                           	std::bind(&DialogSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
  
	m_haltService = m_node->create_service<bt_interfaces_dummy::srv::HaltAction>(m_name + "Skill/halt",
                                                                            	std::bind(&DialogSkill::halt,
                                                                            	this,
                                                                            	std::placeholders::_1,
                                                                            	std::placeholders::_2));
  
  
  
  m_stateMachine.connectToEvent("SchedulerComponent.GetCurrentLanguage.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeGetCurrentLanguage = rclcpp::Node::make_shared(m_name + "SkillNodeGetCurrentLanguage");
      std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::GetCurrentLanguage>> clientGetCurrentLanguage = nodeGetCurrentLanguage->create_client<scheduler_interfaces::srv::GetCurrentLanguage>("/SchedulerComponent/GetCurrentLanguage");
      auto request = std::make_shared<scheduler_interfaces::srv::GetCurrentLanguage::Request>();
      auto eventParams = event.data().toMap();
      
      bool wait_succeded{true};
      int retries = 0;
      while (!clientGetCurrentLanguage->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GetCurrentLanguage'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'GetCurrentLanguage'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientGetCurrentLanguage->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeGetCurrentLanguage, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  data.insert("language", response->language.c_str());
                  
                  m_stateMachine.submitEvent("SchedulerComponent.GetCurrentLanguage.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.GetCurrentLanguage.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'GetCurrentLanguage'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("SchedulerComponent.GetCurrentLanguage.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.GetCurrentLanguage.Return");
  });
  m_stateMachine.connectToEvent("DialogComponent.SetLanguage.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeSetLanguage = rclcpp::Node::make_shared(m_name + "SkillNodeSetLanguage");
      std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::SetLanguage>> clientSetLanguage = nodeSetLanguage->create_client<dialog_interfaces::srv::SetLanguage>("/DialogComponent/SetLanguage");
      auto request = std::make_shared<dialog_interfaces::srv::SetLanguage::Request>();
      auto eventParams = event.data().toMap();
      
      request->new_language = convert<decltype(request->new_language)>(eventParams["new_language"].toString().toStdString());
      bool wait_succeded{true};
      int retries = 0;
      while (!clientSetLanguage->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'SetLanguage'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'SetLanguage'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientSetLanguage->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeSetLanguage, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  m_stateMachine.submitEvent("DialogComponent.SetLanguage.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.SetLanguage.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'SetLanguage'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("DialogComponent.SetLanguage.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.SetLanguage.Return");
  });
  m_stateMachine.connectToEvent("DialogComponent.GetState.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeGetState = rclcpp::Node::make_shared(m_name + "SkillNodeGetState");
      std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::GetState>> clientGetState = nodeGetState->create_client<dialog_interfaces::srv::GetState>("/DialogComponent/GetState");
      auto request = std::make_shared<dialog_interfaces::srv::GetState::Request>();
      auto eventParams = event.data().toMap();
      
      bool wait_succeded{true};
      int retries = 0;
      while (!clientGetState->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GetState'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'GetState'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientGetState->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeGetState, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  data.insert("state", response->state);
                  m_stateMachine.submitEvent("DialogComponent.GetState.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.GetState.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'GetState'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("DialogComponent.GetState.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.GetState.Return");
  });
  m_stateMachine.connectToEvent("DialogComponent.EnableDialog.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeEnableDialog = rclcpp::Node::make_shared(m_name + "SkillNodeEnableDialog");
      std::shared_ptr<rclcpp::Client<dialog_interfaces::srv::EnableDialog>> clientEnableDialog = nodeEnableDialog->create_client<dialog_interfaces::srv::EnableDialog>("/DialogComponent/EnableDialog");
      auto request = std::make_shared<dialog_interfaces::srv::EnableDialog::Request>();
      auto eventParams = event.data().toMap();
      
      request->enable = convert<decltype(request->enable)>(eventParams["enable"].toString().toStdString());
      bool wait_succeded{true};
      int retries = 0;
      while (!clientEnableDialog->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'EnableDialog'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'EnableDialog'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientEnableDialog->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeEnableDialog, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  m_stateMachine.submitEvent("DialogComponent.EnableDialog.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.EnableDialog.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'EnableDialog'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("DialogComponent.EnableDialog.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DialogComponent.EnableDialog.Return");
  });
  
  m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "DialogSkill::tickReturn %s", event.data().toMap()["status"].toString().toStdString().c_str());
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
    RCLCPP_INFO(m_node->get_logger(), "DialogSkill::haltresponse");
    m_haltResult.store(true);
  });

  
  
  
  

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void DialogSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Response>      response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "DialogSkill::tick");
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
  }
  RCLCPP_INFO(m_node->get_logger(), "DialogSkill::tickDone");
  response->is_ok = true;
}

void DialogSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Request> request,
    [[maybe_unused]] std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Response> response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "DialogSkill::halt");
  m_haltResult.store(false);
  m_stateMachine.submitEvent("CMD_HALT");
  while(!m_haltResult.load()) {
      std::this_thread::sleep_for (std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(m_node->get_logger(), "DialogSkill::haltDone");
  response->is_ok = true;
}








