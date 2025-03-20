/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
# pragma once

#include <mutex>
#include <thread>
#include <chrono>
#include <ctime>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <log_interfaces/srv/add_to_log.hpp>

class LogComponent 
{
public:
    LogComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    void publisher(const std::string text);
    void AddToLog( const std::shared_ptr<log_interfaces::srv::AddToLog::Request> request,
                std::shared_ptr<log_interfaces::srv::AddToLog::Response>      response);

private:
    std::string getCurrentTime();
    bool writeInFile(const std::string fileContent);
    bool openFile(const std::string filePath);
    
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<log_interfaces::srv::AddToLog>::SharedPtr m_addToLogService;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscription;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;

    std::string m_logFile;    
    std::mutex m_mutex;

};
