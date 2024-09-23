/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <blackboard_interfaces/srv/get_int_blackboard.hpp>
#include <blackboard_interfaces/srv/set_int_blackboard.hpp>
#include <map>

class BlackboardComponent
{
public:
    BlackboardComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    void GetInt( const std::shared_ptr<blackboard_interfaces::srv::GetIntBlackboard::Request> request,
                std::shared_ptr<blackboard_interfaces::srv::GetIntBlackboard::Response>      response);
    void SetInt( const std::shared_ptr<blackboard_interfaces::srv::SetIntBlackboard::Request> request,
                std::shared_ptr<blackboard_interfaces::srv::SetIntBlackboard::Response>      response);

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<blackboard_interfaces::srv::SetIntBlackboard>::SharedPtr m_setIntService;
    rclcpp::Service<blackboard_interfaces::srv::GetIntBlackboard>::SharedPtr m_getIntService;
    std::mutex m_mutexDouble;
    std::mutex m_mutexInt;
    std::mutex m_mutexString;
    std::map<std::string, int32_t> m_intBlacboard;

};
