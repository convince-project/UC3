/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "BlackboardComponent.h"

bool BlackboardComponent::start(int argc, char*argv[])
{

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("BlackboardComponentNode");
    m_setDoubleService = m_node->create_service<blackboard_interfaces::srv::SetDoubleBlackboard>("/BlackboardComponent/SetDouble",  
                                                                                std::bind(&BlackboardComponent::SetDouble,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getDoubleService = m_node->create_service<blackboard_interfaces::srv::GetDoubleBlackboard>("/BlackboardComponent/GetDouble",  
                                                                                std::bind(&BlackboardComponent::GetDouble,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_setIntService = m_node->create_service<blackboard_interfaces::srv::SetIntBlackboard>("/BlackboardComponent/SetInt",  
                                                                                std::bind(&BlackboardComponent::SetInt,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getIntService = m_node->create_service<blackboard_interfaces::srv::GetIntBlackboard>("/BlackboardComponent/GetInt",  
                                                                                std::bind(&BlackboardComponent::GetInt,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_setStringService = m_node->create_service<blackboard_interfaces::srv::SetStringBlackboard>("/BlackboardComponent/SetString",  
                                                                                std::bind(&BlackboardComponent::SetString,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getStringService = m_node->create_service<blackboard_interfaces::srv::GetStringBlackboard>("/BlackboardComponent/GetString",  
                                                                                std::bind(&BlackboardComponent::GetString,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    RCLCPP_DEBUG(m_node->get_logger(), "BlackboardComponent::start");
    std::cout << "BlackboardComponent::start";        
    return true;

}

bool BlackboardComponent::close()
{
    rclcpp::shutdown();  
    return true;
}

void BlackboardComponent::spin()
{
    rclcpp::spin(m_node);  
}

void BlackboardComponent::GetDouble( const std::shared_ptr<blackboard_interfaces::srv::GetDoubleBlackboard::Request> request,
             std::shared_ptr<blackboard_interfaces::srv::GetDoubleBlackboard::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutexDouble);
    if (request->field_name == "") {
        response->is_ok = false;
        response->error_msg = "missing required field name";
    } else {
        if (!m_doubleBlacboard.contains(request->field_name)) {
            response->is_ok = false;
            response->error_msg = "field not found";
        } else {
            response->value = m_doubleBlacboard.find(request->field_name)->second;  
            response->is_ok = true;
        }
    }
}


void BlackboardComponent::SetDouble( const std::shared_ptr<blackboard_interfaces::srv::SetDoubleBlackboard::Request> request,
             std::shared_ptr<blackboard_interfaces::srv::SetDoubleBlackboard::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutexDouble);
    if (request->field_name == "") {
        response->is_ok = false;
        response->error_msg = "missing required field name";
    } else if (request->value == std::numeric_limits<double>::quiet_NaN()) {
        response->is_ok = false;
        response->error_msg = "missing required value";
    } else {
        if (m_doubleBlacboard.contains(request->field_name)) {
            response->error_msg = "field already present, overwriting";
        } 
        m_doubleBlacboard.insert_or_assign(request->field_name, request->value);  
        response->is_ok = true;
    }
}


void BlackboardComponent::GetString( const std::shared_ptr<blackboard_interfaces::srv::GetStringBlackboard::Request> request,
             std::shared_ptr<blackboard_interfaces::srv::GetStringBlackboard::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutexString);
    if (request->field_name == "") {
        response->is_ok = false;
        response->error_msg = "missing required field name";
    } else if (!m_stringBlacboard.contains(request->field_name)) {
        response->is_ok = false;
        response->error_msg = "field not found";
    } else {
        response->value = m_stringBlacboard.find(request->field_name)->second;  
        response->is_ok = true;
    }
    
}

void BlackboardComponent::SetString(const std::shared_ptr<blackboard_interfaces::srv::SetStringBlackboard::Request> request,
            std::shared_ptr<blackboard_interfaces::srv::SetStringBlackboard::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutexString);
    if (request->field_name == "") {
        response->is_ok = false;
        response->error_msg = "missing required field name";
    } else if (request->value == "") {
        response->is_ok = false;
        response->error_msg = "missing required value";
    } else {
        if (m_stringBlacboard.contains(request->field_name)) {
            response->error_msg = "field already present, overwriting";
        } 
        m_stringBlacboard.insert_or_assign(request->field_name, request->value);  
        response->is_ok = true;
    }
}


void BlackboardComponent::GetInt( const std::shared_ptr<blackboard_interfaces::srv::GetIntBlackboard::Request> request,
             std::shared_ptr<blackboard_interfaces::srv::GetIntBlackboard::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutexInt);
    if (request->field_name == "") {
        response->is_ok = false;
        response->error_msg = "missing required field name";
    } else {
        if (!m_intBlacboard.contains(request->field_name)) {
            response->is_ok = false;
            response->error_msg = "field not found";
        } else {
            response->value = m_intBlacboard.find(request->field_name)->second;  
            response->is_ok = true;
        }
    }
}


void BlackboardComponent::SetInt( const std::shared_ptr<blackboard_interfaces::srv::SetIntBlackboard::Request> request,
             std::shared_ptr<blackboard_interfaces::srv::SetIntBlackboard::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutexInt);
    if (request->field_name == "") {
        response->is_ok = false;
        response->error_msg = "missing required field name";
    } else {
        if (m_intBlacboard.contains(request->field_name)) {
            response->error_msg = "field already present, overwriting";
        } 
        m_intBlacboard.insert_or_assign(request->field_name, request->value);  
        response->is_ok = true;
    }
}
