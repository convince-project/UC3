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
    m_setAllIntsWithPrefixService = m_node->create_service<blackboard_interfaces::srv::SetAllIntsWithPrefixBlackboard>("/BlackboardComponent/SetAllIntsWithPrefix",  
                                                                                std::bind(&BlackboardComponent::SetAllIntsWithPrefix,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    RCLCPP_DEBUG(m_node->get_logger(), "BlackboardComponent::start");
    std::cout << "BlackboardComponent::start" << std::endl;        
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
        if (!m_doubleBlackboard.contains(request->field_name)) {
            response->is_ok = false;
            response->error_msg = "field not found";
        } else {
            response->value = m_doubleBlackboard.find(request->field_name)->second;  
            std::cout << "GetDouble: " << request->field_name << " " << response->value << std::endl; 
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
        if (m_doubleBlackboard.contains(request->field_name)) {
            response->error_msg = "field already present, overwriting";
        } 
        m_doubleBlackboard.insert_or_assign(request->field_name, request->value);  
        std::cout << "SetDouble: " << request->field_name << " " << request->value << std::endl;
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
    } else if (!m_stringBlackboard.contains(request->field_name)) {
        response->is_ok = false;
        response->error_msg = "field not found";
    } else {
        response->value = m_stringBlackboard.find(request->field_name)->second;  
        std::cout << "GetString: " << request->field_name << " " << response->value << std::endl; 
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
        if (m_stringBlackboard.contains(request->field_name)) {
            response->error_msg = "field already present, overwriting";
        } 
        m_stringBlackboard.insert_or_assign(request->field_name, request->value);  
        std::cout << "SetString: " << request->field_name << " " << request->value << std::endl;
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
        if (!m_intBlackboard.contains(request->field_name)) {
            response->is_ok = false;
            response->error_msg = "field not found";
        } else {
            response->value = m_intBlackboard.find(request->field_name)->second; 
            std::cout << "GetInt: " << request->field_name << " " << response->value << std::endl; 
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
        if (m_intBlackboard.contains(request->field_name)) {
            response->error_msg = "field already present, overwriting";
        } 
        m_intBlackboard.insert_or_assign(request->field_name, request->value); 
        std::cout << "SetInt: " << request->field_name << " " << request->value << std::endl; 
        response->is_ok = true;
    }
}

void BlackboardComponent::SetAllIntsWithPrefix(const std::shared_ptr<blackboard_interfaces::srv::SetAllIntsWithPrefixBlackboard::Request> request,
    std::shared_ptr<blackboard_interfaces::srv::SetAllIntsWithPrefixBlackboard::Response> response) 
{
    std::lock_guard<std::mutex> lock(m_mutexInt);
    std::string prefix = request->field_name;
    if (prefix.empty()) {
        response->is_ok = false;
        response->error_msg = "Field name is empty";
        return;
    }
    
    bool foundAny = false;
    for (auto& [key, value] : m_intBlackboard) {
        if (key.find(prefix) == 0) {
            foundAny = true;
            value = request->value; 
            std::cout << "SetAllIntsWithPrefix: " << key << " " << request->value << std::endl;
        }
    }
    
    if (!foundAny) {
        response->is_ok = false;
        response->error_msg = "No fields with the given prefix found";
    } else {
        response->is_ok = true;
    }
}