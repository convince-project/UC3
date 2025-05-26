#include "NarratePoiSkill.h"
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

NarratePoiSkill::NarratePoiSkill(std::string name ) :
		m_name(std::move(name))
{
    
}

void NarratePoiSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool NarratePoiSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "NarratePoiSkill::start");
	std::cout << "NarratePoiSkill::start";

  
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickAction>(m_name + "Skill/tick",
                                                                           	std::bind(&NarratePoiSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
  
	m_haltService = m_node->create_service<bt_interfaces_dummy::srv::HaltAction>(m_name + "Skill/halt",
                                                                            	std::bind(&NarratePoiSkill::halt,
                                                                            	this,
                                                                            	std::placeholders::_1,
                                                                            	std::placeholders::_2));
  
  
  
  m_stateMachine.connectToEvent("NarrateComponent.Stop.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeStop = rclcpp::Node::make_shared(m_name + "SkillNodeStop");
      std::shared_ptr<rclcpp::Client<narrate_interfaces::srv::Stop>> clientStop = nodeStop->create_client<narrate_interfaces::srv::Stop>("/NarrateComponent/Stop");
      auto request = std::make_shared<narrate_interfaces::srv::Stop::Request>();
      auto eventParams = event.data().toMap();
      
      bool wait_succeded{true};
      int retries = 0;
      while (!clientStop->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'Stop'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'Stop'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientStop->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeStop, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  m_stateMachine.submitEvent("NarrateComponent.Stop.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NarrateComponent.Stop.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'Stop'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("NarrateComponent.Stop.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NarrateComponent.Stop.Return");
  });
  m_stateMachine.connectToEvent("NarrateComponent.Narrate.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeNarrate = rclcpp::Node::make_shared(m_name + "SkillNodeNarrate");
      std::shared_ptr<rclcpp::Client<narrate_interfaces::srv::Narrate>> clientNarrate = nodeNarrate->create_client<narrate_interfaces::srv::Narrate>("/NarrateComponent/Narrate");
      auto request = std::make_shared<narrate_interfaces::srv::Narrate::Request>();
      auto eventParams = event.data().toMap();
      
      request->command = convert<decltype(request->command)>(eventParams["command"].toString().toStdString());
      bool wait_succeded{true};
      int retries = 0;
      while (!clientNarrate->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'Narrate'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'Narrate'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientNarrate->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeNarrate, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  m_stateMachine.submitEvent("NarrateComponent.Narrate.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NarrateComponent.Narrate.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'Narrate'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("NarrateComponent.Narrate.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NarrateComponent.Narrate.Return");
  });
  m_stateMachine.connectToEvent("NarrateComponent.IsDone.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> nodeIsDone = rclcpp::Node::make_shared(m_name + "SkillNodeIsDone");
      std::shared_ptr<rclcpp::Client<narrate_interfaces::srv::IsDone>> clientIsDone = nodeIsDone->create_client<narrate_interfaces::srv::IsDone>("/NarrateComponent/IsDone");
      auto request = std::make_shared<narrate_interfaces::srv::IsDone::Request>();
      auto eventParams = event.data().toMap();
      
      bool wait_succeded{true};
      int retries = 0;
      while (!clientIsDone->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'IsDone'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'IsDone'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = clientIsDone->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete(nodeIsDone, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);
                  data.insert("is_done", response->is_done);
                  m_stateMachine.submitEvent("NarrateComponent.IsDone.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NarrateComponent.IsDone.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'IsDone'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("NarrateComponent.IsDone.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NarrateComponent.IsDone.Return");
  });
  
  m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "NarratePoiSkill::tickReturn %s", event.data().toMap()["status"].toString().toStdString().c_str());
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
    RCLCPP_INFO(m_node->get_logger(), "NarratePoiSkill::haltresponse");
    m_haltResult.store(true);
  });

  
  
  
  

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void NarratePoiSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Response>      response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "NarratePoiSkill::tick");
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
  RCLCPP_INFO(m_node->get_logger(), "NarratePoiSkill::tickDone");
  response->is_ok = true;
}

void NarratePoiSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Request> request,
    [[maybe_unused]] std::shared_ptr<bt_interfaces_dummy::srv::HaltAction::Response> response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "NarratePoiSkill::halt");
  m_haltResult.store(false);
  m_stateMachine.submitEvent("CMD_HALT");
  while(!m_haltResult.load()) {
      std::this_thread::sleep_for (std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(m_node->get_logger(), "NarratePoiSkill::haltDone");
  response->is_ok = true;
}








