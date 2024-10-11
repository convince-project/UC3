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
    
    m_setIntService = m_node->create_service<blackboard_interfaces_dummy::srv::SetIntBlackboard>("/BlackboardComponent/SetInt",  
                                                                                std::bind(&BlackboardComponent::SetInt,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getIntService = m_node->create_service<blackboard_interfaces_dummy::srv::GetIntBlackboard>("/BlackboardComponent/GetInt",  
                                                                                std::bind(&BlackboardComponent::GetInt,
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


void BlackboardComponent::GetInt( const std::shared_ptr<blackboard_interfaces_dummy::srv::GetIntBlackboard::Request> request,
             std::shared_ptr<blackboard_interfaces_dummy::srv::GetIntBlackboard::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutexInt);
    std::cout << "GetInt: " << std::endl;
    std::string field_name = "PoiDone" + std::to_string(request->field_name);
    std::cout << "Request: " << request->field_name << "translation " << field_name << std::endl; 
    if (field_name == "PoiDone") {
        response->is_ok = false;
        // response->error_msg = "missing required field name";
        std::cout << "GetInt: " << "missing required field name" << std::endl;
    } else {
        if (!m_intBlacboard.contains(field_name)) {
            response->is_ok = false;
            // response->error_msg = "field not found";
            std::cout << "GetInt: " << "field not found" << std::endl;
        } else {
            response->value = m_intBlacboard.find(field_name)->second; 
            std::cout << "GetInt: " << field_name << " " << response->value << std::endl; 
            response->is_ok = true;
        }
    }
}


void BlackboardComponent::SetInt( const std::shared_ptr<blackboard_interfaces_dummy::srv::SetIntBlackboard::Request> request,
             std::shared_ptr<blackboard_interfaces_dummy::srv::SetIntBlackboard::Response>      response) 
{
    std::lock_guard<std::mutex> lock(m_mutexInt);
    std::string field_name = "PoiDone" + std::to_string(request->field_name);
    std::cout << " SetInt Request: " << request->field_name << " translation " << field_name << std::endl; 
    if (field_name == "PoiDone") {
        response->is_ok = false;
        // response->error_msg = "missing required field name";
        std::cout << "SetInt: " << "missing required field name" << std::endl;
    } else {
        if (m_intBlacboard.contains(field_name)) {
            // response->error_msg = "field already present, overwriting";
            std::cout << "SetInt: " << "field already present, overwriting" << std::endl;
        } 
        m_intBlacboard.insert_or_assign(field_name, request->value); 
        std::cout << "SetInt: " << field_name << " " << request->value << std::endl; 
        response->is_ok = true;
    }
    response->is_ok = true;

}
