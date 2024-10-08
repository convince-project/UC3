#include "VisitorsFollowingRobotSkill.h"
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

VisitorsFollowingRobotSkill::VisitorsFollowingRobotSkill(std::string name ) :
		m_name(std::move(name))
{
    
}

void VisitorsFollowingRobotSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool VisitorsFollowingRobotSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "VisitorsFollowingRobotSkill::start");
	std::cout << "VisitorsFollowingRobotSkill::start";

    
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickCondition>(m_name + "Skill/tick",
                                                                           	std::bind(&VisitorsFollowingRobotSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
    

    
    m_subscription_is_followed = m_node->create_subscription<std_msgs::msg::Bool>(
		"/is_followed", 10, std::bind(&VisitorsFollowingRobotSkill::topic_callback_is_followed, this, std::placeholders::_1));
    

    
    
	m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
		RCLCPP_INFO(m_node->get_logger(), "VisitorsFollowingRobotSkill::tickReturn %s", event.data().toMap()["result"].toString().toStdString().c_str());
		std::string result = event.data().toMap()["result"].toString().toStdString();
		if (result == "SUCCESS" )
		{
			m_tickResult.store(Status::success);
		}
		else if (result == "FAILURE" )
		{ 
			m_tickResult.store(Status::failure);
		}
	});
    

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void VisitorsFollowingRobotSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickCondition::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickCondition::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "VisitorsFollowingRobotSkill::tick");
    auto message = bt_interfaces_dummy::msg::ConditionResponse();
    m_tickResult.store(Status::undefined);
    m_stateMachine.submitEvent("CMD_TICK");
   
    while(m_tickResult.load()== Status::undefined) {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
    }
    switch(m_tickResult.load()) 
    {
        
        case Status::failure:
            response->status.status = message.SKILL_FAILURE;
            break;
        case Status::success:
            response->status.status = message.SKILL_SUCCESS;
            break;            
    }
    RCLCPP_INFO(m_node->get_logger(), "VisitorsFollowingRobotSkill::tickDone");
    response->is_ok = true;
}




void VisitorsFollowingRobotSkill::topic_callback_is_followed(const std_msgs::msg::Bool::SharedPtr msg) {
    std::cout << "callback" << std::endl;
    QVariantMap data;
    data.insert("is_followed", msg->data);

    m_stateMachine.submitEvent("PeopleDetectorFilterComponent.is_followed.Sub", data);
    RCLCPP_INFO(m_node->get_logger(), "PeopleDetectorFilterComponent.is_followed.Sub");
}
