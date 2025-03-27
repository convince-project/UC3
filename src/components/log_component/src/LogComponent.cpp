/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "LogComponent.h"


bool LogComponent::start(int argc, char*argv[])
{
    if (argc >= 2)
    {
        if(!openFile(std::string(argv[1])))
        {
            return false;
        }
    }
    else
    {
        std::cerr << "Error: output path is missing" << std::endl;
        return false;
    }
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("LogComponentNode");
    m_addToLogService = m_node->create_service<log_interfaces::srv::AddToLog>("/LogComponent/AddToLog",  
                                                                                std::bind(&LogComponent::AddToLog,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));

    RCLCPP_DEBUG(m_node->get_logger(), "LogComponent::start");
    m_subscription = m_node->create_subscription<std_msgs::msg::String>(
		"/LogComponent/add_to_log", 10, std::bind(&LogComponent::topic_callback, this, std::placeholders::_1));
    m_publisher = m_node->create_publisher<std_msgs::msg::String>("/LogComponent/read_log", 10);

    std::cout << "LogComponent::start\n";        
    return true;

}


bool LogComponent::close()
{
    rclcpp::shutdown(); 
    return true;
}

void LogComponent::spin()
{
    rclcpp::spin(m_node);  
}

void LogComponent::AddToLog(const std::shared_ptr<log_interfaces::srv::AddToLog::Request> request,
             std::shared_ptr<log_interfaces::srv::AddToLog::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "LogComponent::AddToLog text: " << request->text);
    std::string text = request->text;
    publisher(text);
    response->is_ok = true;
}

void LogComponent::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string text = msg->data;
    RCLCPP_INFO_STREAM(m_node->get_logger(), "LogComponent::add_to_log text: " << text);
    publisher(text);
}

void LogComponent::publisher(const std::string text)
{
    std_msgs::msg::String msg;
    std::string time = getCurrentTime();
    std::string output = time + " " + text;
    msg.data = output;
    writeInFile(output);
    m_publisher->publish(msg);
}

std::string LogComponent::getCurrentTime()
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&now_time);
    std::ostringstream oss;
    oss << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S");
    std::string time = oss.str();
    // std::cout << "Current time: " << time << std::endl;
    return time;
}

bool LogComponent::openFile(const std::string filePath)
{
    std::string time = getCurrentTime();
    std::string fileName = filePath + "/" +time +".txt";
    std::ofstream outputFile(fileName);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open file for writing: " << fileName << std::endl;
        return false;
    }
    std::cout<< "Log file: " << fileName << std::endl;
    m_logFile = fileName;
    outputFile.close();
    return true;
}

bool LogComponent::writeInFile(const std::string fileContent)
{
    std::ofstream outputFile(m_logFile, std::ios::app);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open file for writing: " << m_logFile << std::endl;
        return false;
    }
    outputFile << fileContent;
    outputFile << "\n";
    outputFile.close();
    return true;
}