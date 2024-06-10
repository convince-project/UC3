/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "TurnBackManagerComponent.h"


bool TurnBackManagerComponent::start(int argc, char*argv[])
{

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("TurnBackManagerComponentNode");
    m_setMaxTurnBacksService = m_node->create_service<turn_back_manager_interfaces::srv::SetMaxTurnBacks>("/TurnBackManagerComponent/SetMaxTurnBacks",  
                                                                                std::bind(&TurnBackManagerComponent::SetMaxTurnBacks,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getMaxTurnBacksService = m_node->create_service<turn_back_manager_interfaces::srv::GetMaxTurnBacks>("/TurnBackManagerComponent/GetMaxTurnBacks",  
                                                                                std::bind(&TurnBackManagerComponent::GetMaxTurnBacks,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_setMaxConsecutiveFalsesService = m_node->create_service<turn_back_manager_interfaces::srv::SetMaxConsecutiveFalses>("/TurnBackManagerComponent/SetMaxConsecutiveFalses",  
                                                                                std::bind(&TurnBackManagerComponent::SetMaxConsecutiveFalses,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getMaxConsecutiveFalsesService = m_node->create_service<turn_back_manager_interfaces::srv::GetMaxConsecutiveFalses>("/TurnBackManagerComponent/GetMaxConsecutiveFalses",  
                                                                                std::bind(&TurnBackManagerComponent::GetMaxConsecutiveFalses,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_resetCountersService = m_node->create_service<turn_back_manager_interfaces::srv::ResetCounters>("/TurnBackManagerComponent/ResetCounters",  
                                                                                std::bind(&TurnBackManagerComponent::ResetCounters,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_increaseTurnBacksCounterService = m_node->create_service<turn_back_manager_interfaces::srv::IncreaseTurnBacksCounter>("/TurnBackManagerComponent/IncreaseTurnBacksCounter",  
                                                                                std::bind(&TurnBackManagerComponent::IncreaseTurnBacksCounter,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_isAllowedToTurnBackService = m_node->create_service<turn_back_manager_interfaces::srv::IsAllowedToTurnBack>("/TurnBackManagerComponent/IsAllowedToTurnBack",  
                                                                                std::bind(&TurnBackManagerComponent::IsAllowedToTurnBack,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));

    RCLCPP_DEBUG(m_node->get_logger(), "TurnBackManagerComponent::start");
    m_subscriptionPeopleDetector = m_node->create_subscription<people_detector_filter_interfaces::msg::FilterStatus>(
		"/PeopleDetectorFilterComponent/filtered_detection", 10, std::bind(&TurnBackManagerComponent::topic_callback_people_detector, this, std::placeholders::_1));
    std::cout << "TurnBackManagerComponent::start";        
    return true;

}


bool TurnBackManagerComponent::close()
{
    rclcpp::shutdown(); 
    return true;
}

void TurnBackManagerComponent::spin()
{
    rclcpp::spin(m_node);  
}


void TurnBackManagerComponent::SetMaxTurnBacks(const std::shared_ptr<turn_back_manager_interfaces::srv::SetMaxTurnBacks::Request> request,
             std::shared_ptr<turn_back_manager_interfaces::srv::SetMaxTurnBacks::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "TurnBackManagerComponent::SetMaxTurnBacks value:" << request->max);
    m_mutex.lock();
    m_maxNumberTurnBacks = request->max;
    m_mutex.unlock();
    response->is_ok = true;
}

void TurnBackManagerComponent::GetMaxTurnBacks([[maybe_unused]]const std::shared_ptr<turn_back_manager_interfaces::srv::GetMaxTurnBacks::Request> request,
             std::shared_ptr<turn_back_manager_interfaces::srv::GetMaxTurnBacks::Response>      response) 
{
    m_mutex.lock();
    response->max = m_maxNumberTurnBacks;
    m_mutex.unlock();
    RCLCPP_INFO_STREAM(m_node->get_logger(), "TurnBackManagerComponent::GetMaxTurnBacks value:" << response->max);
    response->is_ok = true;
}
void TurnBackManagerComponent::SetMaxConsecutiveFalses(const std::shared_ptr<turn_back_manager_interfaces::srv::SetMaxConsecutiveFalses::Request> request,
             std::shared_ptr<turn_back_manager_interfaces::srv::SetMaxConsecutiveFalses::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "TurnBackManagerComponent::SetMaxConsecutiveFalses value:" << request->max);
    m_mutex.lock();
    m_maxNumberConsecutiveFalse = request->max;
    m_mutex.unlock();
    response->is_ok = true;
}

void TurnBackManagerComponent::GetMaxConsecutiveFalses([[maybe_unused]]const std::shared_ptr<turn_back_manager_interfaces::srv::GetMaxConsecutiveFalses::Request> request,
             std::shared_ptr<turn_back_manager_interfaces::srv::GetMaxConsecutiveFalses::Response>      response) 
{
    m_mutex.lock();
    response->max = m_maxNumberConsecutiveFalse;
    m_mutex.unlock();
    RCLCPP_INFO_STREAM(m_node->get_logger(), "TurnBackManagerComponent::GetMaxConsecutiveFalses value:" << response->max);
    response->is_ok = true;
}

void TurnBackManagerComponent::ResetCounters([[maybe_unused]]const std::shared_ptr<turn_back_manager_interfaces::srv::ResetCounters::Request> request,
             std::shared_ptr<turn_back_manager_interfaces::srv::ResetCounters::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "TurnBackManagerComponent::ResetCounters ");
    m_mutex.lock();
    m_countNumberTurnBacks = 0;
    m_countNumberConsecutiveFalse = 0;
    m_mutex.unlock();
    response->is_ok = true;
}

void TurnBackManagerComponent::IncreaseTurnBacksCounter([[maybe_unused]]const std::shared_ptr<turn_back_manager_interfaces::srv::IncreaseTurnBacksCounter::Request> request,
             std::shared_ptr<turn_back_manager_interfaces::srv::IncreaseTurnBacksCounter::Response>      response) 
{
    m_mutex.lock();
    m_countNumberTurnBacks++;
    RCLCPP_INFO_STREAM(m_node->get_logger(), "TurnBackManagerComponent::IncreaseTurnBacksCounter value:" << m_countNumberTurnBacks);
    m_mutex.unlock();
    response->is_ok = true;
}

void TurnBackManagerComponent::IsAllowedToTurnBack([[maybe_unused]]const std::shared_ptr<turn_back_manager_interfaces::srv::IsAllowedToTurnBack::Request> request,
             std::shared_ptr<turn_back_manager_interfaces::srv::IsAllowedToTurnBack::Response>      response) 
{
    m_mutex.lock();
    response->is_allowed = m_countNumberTurnBacks < m_maxNumberTurnBacks;
    m_mutex.unlock();
    RCLCPP_INFO_STREAM(m_node->get_logger(), "TurnBackManagerComponent::IsAllowedToTurnBack value:" << response->is_allowed);
    response->is_ok = true;
}

void TurnBackManagerComponent::topic_callback_people_detector(const people_detector_filter_interfaces::msg::FilterStatus::SharedPtr msg) {
	// people_detector_filter_interfaces::msg::FilterStatus filterStatus = *msg;
    if (msg == nullptr) {
        std::cerr << "Received null message people detector topic" << std::endl;
        return;
    }
    // std::cout << "Topic " << static_cast<int>(msg->status) << std::endl;
}