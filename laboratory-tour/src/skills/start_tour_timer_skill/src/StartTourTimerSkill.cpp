#include "StartTourTimerSkill.h"
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

StartTourTimerSkill::StartTourTimerSkill(std::string name ) :
		m_name(std::move(name))
{
    
}

void StartTourTimerSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool StartTourTimerSkill::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "StartTourTimerSkill::start");
	std::cout << "StartTourTimerSkill::start";

    
	m_tickService = m_node->create_service<bt_interfaces::srv::TickAction>(m_name + "Skill/tick",
                                                                           	std::bind(&StartTourTimerSkill::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));
    
	m_haltService = m_node->create_service<bt_interfaces::srv::HaltAction>(m_name + "Skill/halt",
                                                                            	std::bind(&StartTourTimerSkill::halt,
                                                                            	this,
                                                                            	std::placeholders::_1,
                                                                            	std::placeholders::_2));
    
    m_stateMachine.connectToEvent("TimeComponent.StartTourTimer.Call", [this]([[maybe_unused]]const QScxmlEvent & event){
        std::shared_ptr<rclcpp::Node> nodeStartTourTimer = rclcpp::Node::make_shared(m_name + "SkillNodeStartTourTimer");
        std::shared_ptr<rclcpp::Client<time_interfaces::srv::StartTourTimer>> clientStartTourTimer = nodeStartTourTimer->create_client<time_interfaces::srv::StartTourTimer>("/TimeComponent/StartTourTimer");
        auto request = std::make_shared<time_interfaces::srv::StartTourTimer::Request>();
        auto eventParams = event.data().toMap();
        
        bool wait_succeded{true};
        int retries = 0;
        while (!clientStartTourTimer->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'StartTourTimer'. Exiting.");
                wait_succeded = false;
                break;
            } 
            retries++;
            if(retries == SERVICE_TIMEOUT) {
               RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'StartTourTimer'.");
               wait_succeded = false;
               break;
            }
        }
        if (wait_succeded) {                                                                   
            auto result = clientStartTourTimer->async_send_request(request);
            const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
            auto futureResult = rclcpp::spin_until_future_complete(nodeStartTourTimer, result, timeout_duration);
            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
            {
               auto response = result.get();
               if( response->is_ok ==true) {
                   QVariantMap data;
                   data.insert("result", "SUCCESS");
                   m_stateMachine.submitEvent("TimeComponent.StartTourTimer.Return", data);
                   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TimeComponent.StartTourTimer.Return");
                   return;
               }
           }
           else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
               RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'StartTourTimer'.");
           }
        }
       QVariantMap data;
       data.insert("result", "FAILURE");
       m_stateMachine.submitEvent("TimeComponent.StartTourTimer.Return", data);
       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TimeComponent.StartTourTimer.Return");
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
               if( response->is_ok ==true) {
                   QVariantMap data;
                   data.insert("result", "SUCCESS");
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
       data.insert("result", "FAILURE");
       m_stateMachine.submitEvent("BlackboardComponent.SetInt.Return", data);
       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BlackboardComponent.SetInt.Return");
    });
    
	m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
		RCLCPP_INFO(m_node->get_logger(), "StartTourTimerSkill::tickReturn %s", event.data().toMap()["result"].toString().toStdString().c_str());
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
		RCLCPP_INFO(m_node->get_logger(), "StartTourTimerSkill::haltresponse");
		m_haltResult.store(true);
	});

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}

void StartTourTimerSkill::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
                                std::shared_ptr<bt_interfaces::srv::TickAction::Response>      response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "StartTourTimerSkill::tick");
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
    RCLCPP_INFO(m_node->get_logger(), "StartTourTimerSkill::tickDone");
    response->is_ok = true;
}

void StartTourTimerSkill::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::HaltAction::Request> request,
    [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::HaltAction::Response> response)
{
    std::lock_guard<std::mutex> lock(m_requestMutex);
    RCLCPP_INFO(m_node->get_logger(), "StartTourTimerSkill::halt");
    m_haltResult.store(false);
    m_stateMachine.submitEvent("CMD_HALT");
    while(!m_haltResult.load()) {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(m_node->get_logger(), "StartTourTimerSkill::haltDone");
    response->is_ok = true;
}
