#include "SetPoi2Skill.h"
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

SetPoi2Skill::SetPoi2Skill(std::string name ) :
		m_name(std::move(name))
{
    
}

void SetPoi2Skill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool SetPoi2Skill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "SetPoi2Skill::start");
	std::cout << "SetPoi2Skill::start";

    
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::TickAction>(m_name + "Skill/tick",
                                                                           	std::bind(&SetPoi2Skill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
    

    

    
    m_stateMachine.connectToEvent("SchedulerComponent.SetPoi.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeSetPoi = rclcpp::Node::make_shared(m_name + "SkillNodeSetPoi");
        std::shared_ptr<rclcpp::Client<scheduler_interfaces_dummy::srv::SetPoi>> clientSetPoi = nodeSetPoi->create_client<scheduler_interfaces_dummy::srv::SetPoi>("/SchedulerComponent/SetPoi");
        auto request = std::make_shared<scheduler_interfaces_dummy::srv::SetPoi::Request>();
        auto eventParams = event.data().toMap();
        
        request->poi_number = convert<decltype(request->poi_number)>(eventParams["poi_number"].toString().toStdString());
        bool wait_succeded{true};
        int retries = 0;
        while (!clientSetPoi->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'SetPoi'. Exiting.");
                wait_succeded = false;
                break;
            } 
            retries++;
            if(retries == SERVICE_TIMEOUT) {
               RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'SetPoi'.");
               wait_succeded = false;
               break;
            }
        }
        if (wait_succeded) {                                                                   
            auto result = clientSetPoi->async_send_request(request);
            const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
            auto futureResult = rclcpp::spin_until_future_complete(nodeSetPoi, result, timeout_duration);
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
               auto response = result.get();
               if( response->is_ok ==true) {
                   QVariantMap data;
                   data.insert("status", SKILL_SUCCESS);
                   m_stateMachine.submitEvent("SchedulerComponent.SetPoi.Return", data);
                   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.SetPoi.Return");
                   return;
               }
           }
           else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
               RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'SetPoi'.");
           }
        }
       QVariantMap data;
       data.insert("status", SKILL_FAILURE);
       m_stateMachine.submitEvent("SchedulerComponent.SetPoi.Return", data);
       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SchedulerComponent.SetPoi.Return");
    });
    
	m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
		RCLCPP_INFO(m_node->get_logger(), "SetPoi2Skill::tickReturn %s", event.data().toMap()["status"].toString().toStdString().c_str());
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

void SetPoi2Skill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::TickAction::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "SetPoi2Skill::tick");
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
    }
    RCLCPP_INFO(m_node->get_logger(), "SetPoi2Skill::tickDone");
    response->is_ok = true;
}



