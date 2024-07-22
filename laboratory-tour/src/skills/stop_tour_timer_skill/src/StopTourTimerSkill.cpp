#include "StopTourTimerSkill.h"
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

StopTourTimerSkill::StopTourTimerSkill(std::string name ) :
		m_name(std::move(name))
{
    
}

void StopTourTimerSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool StopTourTimerSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "StopTourTimerSkill::start");
	std::cout << "StopTourTimerSkill::start";

    
	m_tickService = m_node->create_service<bt_interfaces::srv::TickAction>(m_name + "Skill/tick",
                                                                           	std::bind(&StopTourTimerSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
    
	m_haltService = m_node->create_service<bt_interfaces::srv::HaltAction>(m_name + "Skill/halt",
                                                                            	std::bind(&StopTourTimerSkill::halt,
                                                                            	this,
                                                                            	std::placeholders::_1,
                                                                            	std::placeholders::_2));
    
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
               if( response->is_ok ==true) {
                   QVariantMap data;
                   data.insert("result", "SUCCESS");
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
       data.insert("result", "FAILURE");
       m_stateMachine.submitEvent("TimeComponent.StopTourTimer.Return", data);
       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TimeComponent.StopTourTimer.Return");
    });
    
	m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
		RCLCPP_INFO(m_node->get_logger(), "StopTourTimerSkill::tickReturn %s", event.data().toMap()["result"].toString().toStdString().c_str());
		std::string result = event.data().toMap()["result"].toString().toStdString();
		if (result == "SUCCESS" )
		{
			m_tickResult.store(Status::success);
		}
		else if (result == "RUNNING" )
		{
			m_tickResult.store(Status::running);
		}
		else if (result == "FAILURE" )
		{ 
			m_tickResult.store(Status::failure);
		}
	});
    
	m_stateMachine.connectToEvent("HALT_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
		RCLCPP_INFO(m_node->get_logger(), "StopTourTimerSkill::haltresponse");
		m_haltResult.store(true);
	});

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void StopTourTimerSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
                                std::shared_ptr<bt_interfaces::srv::TickAction::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "StopTourTimerSkill::tick");
    auto message = bt_interfaces::msg::ActionResponse();
    m_tickResult.store(Status::undefined);
    m_stateMachine.submitEvent("CMD_TICK");
   
    while(m_tickResult.load()== Status::undefined) {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
    }
    switch(m_tickResult.load()) 
    {
        case Status::running:
            response->status.status = message.SKILL_RUNNING;
            break;
        case Status::failure:
            response->status.status = message.SKILL_FAILURE;
            break;
        case Status::success:
            response->status.status = message.SKILL_SUCCESS;
            break;            
    }
    RCLCPP_INFO(m_node->get_logger(), "StopTourTimerSkill::tickDone");
    response->is_ok = true;
}

void StopTourTimerSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::HaltAction::Request> request,
    [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::HaltAction::Response> response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "StopTourTimerSkill::halt");
    m_haltResult.store(false);
    m_stateMachine.submitEvent("CMD_HALT");
    while(!m_haltResult.load()) {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(m_node->get_logger(), "StopTourTimerSkill::haltDone");
    response->is_ok = true;
}
