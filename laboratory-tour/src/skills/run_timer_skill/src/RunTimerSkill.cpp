#include "RunTimerSkill.h"
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

RunTimerSkill::RunTimerSkill(std::string name ) :
		m_name(std::move(name))
{
    
}

void RunTimerSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool RunTimerSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "RunTimerSkill::start");
	std::cout << "RunTimerSkill::start";

    
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickAction>(m_name + "Skill/tick",
                                                                           	std::bind(&RunTimerSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
    

    

    
    m_stateMachine.connectToEvent("TimerComponent.StartTimer.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeStartTimer = rclcpp::Node::make_shared(m_name + "SkillNodeStartTimer");
        std::shared_ptr<rclcpp::Client<timer_interfaces_dummy::srv::StartTimer>> clientStartTimer = nodeStartTimer->create_client<timer_interfaces_dummy::srv::StartTimer>("/TimerComponent/StartTimer");
        auto request = std::make_shared<timer_interfaces_dummy::srv::StartTimer::Request>();
        auto eventParams = event.data().toMap();
        
        bool wait_succeded{true};
        int retries = 0;
        while (!clientStartTimer->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'StartTimer'. Exiting.");
                wait_succeded = false;
                break;
            } 
            retries++;
            if(retries == SERVICE_TIMEOUT) {
               RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'StartTimer'.");
               wait_succeded = false;
               break;
            }
        }
        if (wait_succeded) {                                                                   
            auto result = clientStartTimer->async_send_request(request);
            const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
            auto futureResult = rclcpp::spin_until_future_complete(nodeStartTimer, result, timeout_duration);
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
               auto response = result.get();
               if( response->is_ok ==true) {
                   QVariantMap data;
                   data.insert("is_ok", true);
                   m_stateMachine.submitEvent("TimerComponent.StartTimer.Return", data);
                   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TimerComponent.StartTimer.Return");
                   return;
               }
           }
           else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
               RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'StartTimer'.");
           }
        }
       QVariantMap data;
       data.insert("is_ok", false);
       m_stateMachine.submitEvent("TimerComponent.StartTimer.Return", data);
       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TimerComponent.StartTimer.Return");
    });
    m_stateMachine.connectToEvent("TimerComponent.IsTimerActive.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeIsTimerActive = rclcpp::Node::make_shared(m_name + "SkillNodeIsTimerActive");
        std::shared_ptr<rclcpp::Client<timer_interfaces_dummy::srv::IsTimerActive>> clientIsTimerActive = nodeIsTimerActive->create_client<timer_interfaces_dummy::srv::IsTimerActive>("/TimerComponent/IsTimerActive");
        auto request = std::make_shared<timer_interfaces_dummy::srv::IsTimerActive::Request>();
        auto eventParams = event.data().toMap();
        
        bool wait_succeded{true};
        int retries = 0;
        while (!clientIsTimerActive->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'IsTimerActive'. Exiting.");
                wait_succeded = false;
                break;
            } 
            retries++;
            if(retries == SERVICE_TIMEOUT) {
               RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'IsTimerActive'.");
               wait_succeded = false;
               break;
            }
        }
        if (wait_succeded) {                                                                   
            auto result = clientIsTimerActive->async_send_request(request);
            const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
            auto futureResult = rclcpp::spin_until_future_complete(nodeIsTimerActive, result, timeout_duration);
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
               auto response = result.get();
               if( response->is_ok ==true) {
                   QVariantMap data;
                   data.insert("is_ok", true);
                   data.insert("is_active", response->is_active);
                   m_stateMachine.submitEvent("TimerComponent.IsTimerActive.Return", data);
                   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TimerComponent.IsTimerActive.Return");
                   return;
               }
           }
           else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
               RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'IsTimerActive'.");
           }
        }
       QVariantMap data;
       data.insert("is_ok", false);
       m_stateMachine.submitEvent("TimerComponent.IsTimerActive.Return", data);
       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TimerComponent.IsTimerActive.Return");
    });
    
	m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
		RCLCPP_INFO(m_node->get_logger(), "RunTimerSkill::tickReturn %s", event.data().toMap()["status"].toString().toStdString().c_str());
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
    

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void RunTimerSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "RunTimerSkill::tick");
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
    RCLCPP_INFO(m_node->get_logger(), "RunTimerSkill::tickDone");
    response->is_ok = true;
}



