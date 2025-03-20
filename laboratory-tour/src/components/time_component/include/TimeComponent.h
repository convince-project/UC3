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
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <time_interfaces/srv/start_tour_timer.hpp>
#include <time_interfaces/srv/stop_tour_timer.hpp>
#include <time_interfaces/srv/reload_config_file.hpp>
#include <time_interfaces/srv/is_museum_closing.hpp>
#include <blackboard_interfaces/srv/set_int_blackboard.hpp>

#define WARNING_BB_STRING   "TourDurationWarningFlag"
#define MAX_BB_STRING       "TourDurationExceededFlag"
#define DURATION_BB_STRING  "TimePassedMinutes"
#define SAID_WARNING_BB_STRING  "SaidWarningFlag"
#define SERVICE_TIMEOUT     2

class TimeComponent 
{
public:
    TimeComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    void publisher(std::string text);
    void StartTourTimer([[maybe_unused]] const std::shared_ptr<time_interfaces::srv::StartTourTimer::Request> request,
                std::shared_ptr<time_interfaces::srv::StartTourTimer::Response>      response);
    void StopTourTimer([[maybe_unused]] const std::shared_ptr<time_interfaces::srv::StopTourTimer::Request> request,
                std::shared_ptr<time_interfaces::srv::StopTourTimer::Response>      response);
    void ReloadConfigFile([[maybe_unused]] const std::shared_ptr<time_interfaces::srv::ReloadConfigFile::Request> request,
                std::shared_ptr<time_interfaces::srv::ReloadConfigFile::Response>      response);
    void IsMuseumClosing([[maybe_unused]] const std::shared_ptr<time_interfaces::srv::IsMuseumClosing::Request> request,
                std::shared_ptr<time_interfaces::srv::IsMuseumClosing::Response>      response);
    

private:
    std::string getCurrentTime();
    bool getParamsFromFile(std::ifstream& file);
    bool loadConfigFile();
    bool getValue(std::unordered_map<std::string, std::unordered_map<std::string, std::string>> config, const std::string section, const std::string key, std::string& value);
    int getTimeInterval(const std::string timeStamp1, const std::string timeStamp2);
    void timerTask();
    bool writeInBB(std::string key, int value);
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<time_interfaces::srv::StartTourTimer>::SharedPtr m_startTourTimerService;
    rclcpp::Service<time_interfaces::srv::StopTourTimer>::SharedPtr m_stopTourTimerService;
    rclcpp::Service<time_interfaces::srv::ReloadConfigFile>::SharedPtr m_reloadConfigFileService;
    rclcpp::Service<time_interfaces::srv::IsMuseumClosing>::SharedPtr m_isMuseumClosingService;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    std::thread m_threadTimer;
    std::mutex m_timerMutex;
    std::mutex m_mutex;
    std::string m_configPath;
    std::string m_closingTime;
    std::string m_openingTime;
    bool m_printMuseumClosing{false};
    bool m_stopped{false};
    int m_warningTime{2};
    int m_maxTime{4};
    bool m_timerTask{false};
    bool m_saidDurationWarning{false};
};
