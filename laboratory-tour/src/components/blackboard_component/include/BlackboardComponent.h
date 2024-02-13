/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <blackboard_interfaces/srv/get_double_blackboard.hpp>
#include <blackboard_interfaces/srv/set_double_blackboard.hpp>
#include <blackboard_interfaces/srv/get_int_blackboard.hpp>
#include <blackboard_interfaces/srv/set_int_blackboard.hpp>
#include <blackboard_interfaces/srv/set_string_blackboard.hpp>
#include <blackboard_interfaces/srv/get_string_blackboard.hpp>
#include <map>

class BlackboardComponent
{
public:
    BlackboardComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    void GetDouble( const std::shared_ptr<blackboard_interfaces::srv::GetDoubleBlackboard::Request> request,
                std::shared_ptr<blackboard_interfaces::srv::GetDoubleBlackboard::Response>      response);
    void SetDouble( const std::shared_ptr<blackboard_interfaces::srv::SetDoubleBlackboard::Request> request,
                std::shared_ptr<blackboard_interfaces::srv::SetDoubleBlackboard::Response>      response);
    void GetInt( const std::shared_ptr<blackboard_interfaces::srv::GetIntBlackboard::Request> request,
                std::shared_ptr<blackboard_interfaces::srv::GetIntBlackboard::Response>      response);
    void SetInt( const std::shared_ptr<blackboard_interfaces::srv::SetIntBlackboard::Request> request,
                std::shared_ptr<blackboard_interfaces::srv::SetIntBlackboard::Response>      response);
    void GetString( const std::shared_ptr<blackboard_interfaces::srv::GetStringBlackboard::Request> request,
                std::shared_ptr<blackboard_interfaces::srv::GetStringBlackboard::Response>      response);
    void SetString( const std::shared_ptr<blackboard_interfaces::srv::SetStringBlackboard::Request> request,
                std::shared_ptr<blackboard_interfaces::srv::SetStringBlackboard::Response>      response);

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<blackboard_interfaces::srv::SetDoubleBlackboard>::SharedPtr m_setDoubleService;
    rclcpp::Service<blackboard_interfaces::srv::GetDoubleBlackboard>::SharedPtr m_getDoubleService;
    rclcpp::Service<blackboard_interfaces::srv::SetIntBlackboard>::SharedPtr m_setIntService;
    rclcpp::Service<blackboard_interfaces::srv::GetIntBlackboard>::SharedPtr m_getIntService;
    rclcpp::Service<blackboard_interfaces::srv::SetStringBlackboard>::SharedPtr m_setStringService;
    rclcpp::Service<blackboard_interfaces::srv::GetStringBlackboard>::SharedPtr m_getStringService;
    std::mutex m_mutexDouble;
    std::mutex m_mutexInt;
    std::mutex m_mutexString;
    std::map<std::string, std::string> m_stringBlacboard;
    std::map<std::string, double> m_doubleBlacboard;
    std::map<std::string, int32_t> m_intBlacboard;

};
