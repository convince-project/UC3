#include "NetworkStatusChangedSkill.h"
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

NetworkStatusChangedSkill::NetworkStatusChangedSkill(std::string name ) :
		m_name(std::move(name))
{
    
}

NetworkStatusChangedSkill::~NetworkStatusChangedSkill()
{
    //std::cout << "DEBUG: Invoked destructor of NetworkStatusChangedSkill" << std::endl;
    m_threadSpin->join();
}

void NetworkStatusChangedSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::spin(node);
    rclcpp::shutdown();
    QCoreApplication::quit();
    //std::cout << "DEBUG: NetworkStatusChangedSkill::spin successfully ended" << std::endl;
}

bool NetworkStatusChangedSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "NetworkStatusChangedSkill::start");
	std::cout << "DEBUG: NetworkStatusChangedSkill::start" << std::endl;

  
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickCondition>(m_name + "Skill/tick",
                                                                           	std::bind(&NetworkStatusChangedSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
  m_tickService->configure_introspection(m_node->get_clock(), rclcpp::SystemDefaultsQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);
  
  
  
  
  
  m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "NetworkStatusChangedSkill::tickReturn %s", event.data().toMap()["status"].toString().toStdString().c_str());
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

void NetworkStatusChangedSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickCondition::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickCondition::Response>      response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "NetworkStatusChangedSkill::tick");
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
  RCLCPP_INFO(m_node->get_logger(), "NetworkStatusChangedSkill::tickDone");
  response->is_ok = true;
}








