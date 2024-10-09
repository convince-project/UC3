#include "BatteryLevelSkill.h"
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

BatteryLevelSkill::BatteryLevelSkill(std::string name ) :
		m_name(std::move(name))
{
    
}

void BatteryLevelSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool BatteryLevelSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "BatteryLevelSkill::start");
	std::cout << "BatteryLevelSkill::start";

    
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickCondition>(m_name + "Skill/tick",
                                                                           	std::bind(&BatteryLevelSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
    

    
    m_subscription_battery_status = m_node->create_subscription<std_msgs::msg::Int32>(
		"/battery_status", 10, std::bind(&BatteryLevelSkill::topic_callback_battery_status, this, std::placeholders::_1));
    

    
    
	m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
		RCLCPP_INFO(m_node->get_logger(), "BatteryLevelSkill::tickReturn %s", event.data().toMap()["status"].toString().toStdString().c_str());
		std::string result = event.data().toMap()["status"].toString().toStdString();
		if (result == message.SKILL_SUCCESS )
		{
			m_tickResult.store(Status::success);
		}
		else if (result == message.SKILL_FAILURE )
		{ 
			m_tickResult.store(Status::failure);
		}
	});
    

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void BatteryLevelSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickCondition::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickCondition::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "BatteryLevelSkill::tick");
    auto message = bt_interfaces_dummy::msg::ConditionResponse();
    m_tickResult.store(Status::undefined);
    m_stateMachine.submitEvent("CMD_TICK");
   
    while(m_tickResult.load()== Status::undefined) {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
    }
    switch(m_tickResult.load()) 
    {
        
        case Status::failure:
            response->status = message.SKILL_FAILURE;
            break;
        case Status::success:
            response->status = message.SKILL_SUCCESS;
            break;            
    }
    RCLCPP_INFO(m_node->get_logger(), "BatteryLevelSkill::tickDone");
    response->is_ok = true;
}




void BatteryLevelSkill::topic_callback_battery_status(const std_msgs::msg::Int32::SharedPtr msg) {
    std::cout << "callback" << std::endl;
    QVariantMap data;
    data.insert("percentage", msg->data);

    m_stateMachine.submitEvent("BatteryComponent.battery_status.Sub", data);
    RCLCPP_INFO(m_node->get_logger(), "BatteryComponent.battery_status.Sub");
}
