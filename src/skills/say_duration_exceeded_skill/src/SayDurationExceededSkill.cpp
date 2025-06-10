#include "SayDurationExceededSkill.h"
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

SayDurationExceededSkill::SayDurationExceededSkill(std::string name ) :
		m_name(std::move(name))
{
    
}

void SayDurationExceededSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool SayDurationExceededSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "SayDurationExceededSkill::start");
	std::cout << "SayDurationExceededSkill::start";

  
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickAction>(m_name + "Skill/tick",
                                                                           	std::bind(&SayDurationExceededSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
  
	m_haltService = m_node->create_service<bt_interfaces_dummy::srv::HaltAction>(m_name + "Skill/halt",
                                                                            	std::bind(&SayDurationExceededSkill::halt,
                                                                            	this,
                                                                            	std::placeholders::_1,
                                                                            	std::placeholders::_2));
  
  
  
  m_stateMachine.connectToEvent("TextToSpeechComponent.Speak.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeSpeak = rclcpp::Node::make_shared(m_name + "SkillNodeSpeak");
      std::shared_ptr<rclcpp::Client<text_to_speech_interfaces::srv::Speak>> clientSpeak = nodeSpeak->create_client<text_to_speech_interfaces::srv::Speak>("/TextToSpeechComponent/Speak");
      auto request = std::make_shared<text_to_speech_interfaces::srv::Speak::Request>();
      auto eventParams = event.data().toMap();
      
      request->text = convert<decltype(request->text)>(eventParams["text"].toString().toStdString());
      bool wait_succeded{true};
      int retries = 0;
      while (!clientSpeak->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'Speak'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'Speak'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientSpeak->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeSpeak, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  m_stateMachine.submitEvent("TextToSpeechComponent.Speak.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TextToSpeechComponent.Speak.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'Speak'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("TextToSpeechComponent.Speak.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TextToSpeechComponent.Speak.Return");
  });
  m_stateMachine.connectToEvent("TextToSpeechComponent.IsSpeaking.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeIsSpeaking = rclcpp::Node::make_shared(m_name + "SkillNodeIsSpeaking");
      std::shared_ptr<rclcpp::Client<text_to_speech_interfaces::srv::IsSpeaking>> clientIsSpeaking = nodeIsSpeaking->create_client<text_to_speech_interfaces::srv::IsSpeaking>("/TextToSpeechComponent/IsSpeaking");
      auto request = std::make_shared<text_to_speech_interfaces::srv::IsSpeaking::Request>();
      auto eventParams = event.data().toMap();
      
      bool wait_succeded{true};
      int retries = 0;
      while (!clientIsSpeaking->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'IsSpeaking'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'IsSpeaking'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientIsSpeaking->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeIsSpeaking, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  data.insert("is_speaking", response->is_speaking);
                  m_stateMachine.submitEvent("TextToSpeechComponent.IsSpeaking.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TextToSpeechComponent.IsSpeaking.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'IsSpeaking'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("TextToSpeechComponent.IsSpeaking.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TextToSpeechComponent.IsSpeaking.Return");
  });
  m_stateMachine.connectToEvent("SchedulerComponent.SetCommand.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeSetCommand = rclcpp::Node::make_shared(m_name + "SkillNodeSetCommand");
      std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::SetCommand>> clientSetCommand = nodeSetCommand->create_client<scheduler_interfaces::srv::SetCommand>("/SchedulerComponent/SetCommand");
      auto request = std::make_shared<scheduler_interfaces::srv::SetCommand::Request>();
      auto eventParams = event.data().toMap();
      
      request->command = convert<decltype(request->command)>(eventParams["command"].toString().toStdString());
      bool wait_succeded{true};
      int retries = 0;
      while (!clientSetCommand->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'SetCommand'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'SetCommand'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientSetCommand->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeSetCommand, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  m_stateMachine.submitEvent("SchedulerComponent.SetCommand.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.SetCommand.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'SetCommand'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("SchedulerComponent.SetCommand.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.SetCommand.Return");
  });
  m_stateMachine.connectToEvent("SchedulerComponent.GetCurrentAction.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeGetCurrentAction = rclcpp::Node::make_shared(m_name + "SkillNodeGetCurrentAction");
      std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::GetCurrentAction>> clientGetCurrentAction = nodeGetCurrentAction->create_client<scheduler_interfaces::srv::GetCurrentAction>("/SchedulerComponent/GetCurrentAction");
      auto request = std::make_shared<scheduler_interfaces::srv::GetCurrentAction::Request>();
      auto eventParams = event.data().toMap();
      
      bool wait_succeded{true};
      int retries = 0;
      while (!clientGetCurrentAction->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GetCurrentAction'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'GetCurrentAction'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientGetCurrentAction->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeGetCurrentAction, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  data.insert("type", response->type.c_str());
                  data.insert("param", response->param.c_str());
                  data.insert("is_blocking", response->is_blocking);
                  m_stateMachine.submitEvent("SchedulerComponent.GetCurrentAction.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.GetCurrentAction.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'GetCurrentAction'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("SchedulerComponent.GetCurrentAction.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.GetCurrentAction.Return");
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
  m_stateMachine.connectToEvent("BlackboardComponent.GetInt.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeGetInt = rclcpp::Node::make_shared(m_name + "SkillNodeGetInt");
      std::shared_ptr<rclcpp::Client<blackboard_interfaces::srv::GetIntBlackboard>> clientGetInt = nodeGetInt->create_client<blackboard_interfaces::srv::GetIntBlackboard>("/BlackboardComponent/GetInt");
      auto request = std::make_shared<blackboard_interfaces::srv::GetIntBlackboard::Request>();
      auto eventParams = event.data().toMap();
      
      request->field_name = convert<decltype(request->field_name)>(eventParams["field_name"].toString().toStdString());
      bool wait_succeded{true};
      int retries = 0;
      while (!clientGetInt->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GetInt'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'GetInt'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientGetInt->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeGetInt, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  data.insert("value", response->value);
                  m_stateMachine.submitEvent("BlackboardComponent.GetInt.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BlackboardComponent.GetInt.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'GetInt'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("BlackboardComponent.GetInt.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BlackboardComponent.GetInt.Return");
  });
  
  m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "SayDurationExceededSkill::tickReturn %s", event.data().toMap()["status"].toString().toStdString().c_str());
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
    RCLCPP_INFO(m_node->get_logger(), "SayDurationExceededSkill::haltresponse");
    m_haltResult.store(true);
  });

  
  
  
  

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void SayDurationExceededSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Response>      response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "SayDurationExceededSkill::tick");
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
          response->status = SKILL_FAILURE; // Default case, should not happen
          break;
  }
  RCLCPP_INFO(m_node->get_logger(), "SayDurationExceededSkill::tickDone");
  response->is_ok = true;
}

void SayDurationExceededSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Request> request,
    [[maybe_unused]] std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Response> response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "SayDurationExceededSkill::halt");
  m_haltResult.store(false);
  m_stateMachine.submitEvent("CMD_HALT");
  while(!m_haltResult.load()) {
      std::this_thread::sleep_for (std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(m_node->get_logger(), "SayDurationExceededSkill::haltDone");
  response->is_ok = true;
}








