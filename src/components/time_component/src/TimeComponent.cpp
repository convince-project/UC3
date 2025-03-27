/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "TimeComponent.h"


bool TimeComponent::start(int argc, char*argv[])
{
    if (argc >= 2)
    {
        m_configPath = argv[1];
        if(!loadConfigFile())
        {
            return false;
        }
    }
    else
    {
        std::cerr << "Error: config.ini path is missing" << std::endl;
        return false;
    }

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("TimeComponentNode");
    m_startTourTimerService = m_node->create_service<time_interfaces::srv::StartTourTimer>("/TimeComponent/StartTourTimer",  
                                                                                std::bind(&TimeComponent::StartTourTimer,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_stopTourTimerService = m_node->create_service<time_interfaces::srv::StopTourTimer>("/TimeComponent/StopTourTimer",  
                                                                                std::bind(&TimeComponent::StopTourTimer,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_reloadConfigFileService = m_node->create_service<time_interfaces::srv::ReloadConfigFile>("/TimeComponent/ReloadConfigFile",  
                                                                                std::bind(&TimeComponent::ReloadConfigFile,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_isMuseumClosingService = m_node->create_service<time_interfaces::srv::IsMuseumClosing>("/TimeComponent/IsMuseumClosing",  
                                                                                std::bind(&TimeComponent::IsMuseumClosing,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));

    RCLCPP_DEBUG(m_node->get_logger(), "TimeComponent::start");
    m_publisher = m_node->create_publisher<std_msgs::msg::String>("/LogComponent/add_to_log", 10);
    std::cout << "TimeComponent::start\n";        
    return true;

}


bool TimeComponent::close()
{
    rclcpp::shutdown(); 
    return true;
}

void TimeComponent::spin()
{
    rclcpp::spin(m_node);  
}

void TimeComponent::StartTourTimer([[maybe_unused]]const std::shared_ptr<time_interfaces::srv::StartTourTimer::Request> request,
             std::shared_ptr<time_interfaces::srv::StartTourTimer::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "TimeComponent::StartTourTimer ");
    m_timerMutex.lock();
    if(m_timerTask == true)
    {
        RCLCPP_INFO_STREAM(m_node->get_logger(), "Timer task already running");
        response->is_ok = true;
        return;
    }
    m_timerMutex.unlock();
    m_saidDurationWarning = false;
    m_threadTimer = std::thread([this]() { timerTask(); });
    response->is_ok = true;
}

void TimeComponent::ReloadConfigFile([[maybe_unused]]const std::shared_ptr<time_interfaces::srv::ReloadConfigFile::Request> request, 
            std::shared_ptr<time_interfaces::srv::ReloadConfigFile::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "TimeComponent::ReloadConfigFile ");
    if(!loadConfigFile())
    {
        response->is_ok = false;
        return;
    }
    response->is_ok = true;
}

void TimeComponent::IsMuseumClosing([[maybe_unused]]const std::shared_ptr<time_interfaces::srv::IsMuseumClosing::Request> request, 
            std::shared_ptr<time_interfaces::srv::IsMuseumClosing::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "TimeComponent::IsMuseumClosing ");
    std::string l_currentTime  = getCurrentTime();
    m_mutex.lock();
    int l_maxTime = m_maxTime;
    int l_timeIntervalClosing = getTimeInterval(l_currentTime, m_closingTime);// >0 if currentTime is before opening hours,  <0 if currentTime is after opening hours
    int l_timeIntervalOpening = getTimeInterval(m_openingTime, l_currentTime);// <0 if currentTime is before opening hours,  >0 if currentTime is after opening hours
    m_mutex.unlock();
    response->is_closing = (l_timeIntervalClosing <= l_maxTime) ||  l_timeIntervalOpening < 0;
    if(response->is_closing)
    {
        std::cout << "Museum is closing/closed! " <<std::endl;
        if(l_timeIntervalOpening < 0)
        {
            std::cout << "Time until opening hours: "<< abs(l_timeIntervalOpening)/60 << "h" << abs(l_timeIntervalOpening)%60 << "m" <<std::endl;
        }
        else
        {
            std::cout << "Time after closing hours: "<< abs(l_timeIntervalClosing)/60 << "h" << abs(l_timeIntervalClosing)%60 << "m" <<std::endl;
        }
        
        if(!m_printMuseumClosing)
        {
            publisher("Museum is closing soon!");
            m_printMuseumClosing = true;
        }
    }
    else
    {
        std::cout << "Time until closing hours: "<< abs(l_timeIntervalClosing)/60 << "h" << abs(l_timeIntervalClosing)%60 << "m" <<std::endl;
        std::cout << "Time after opening hours: "<< abs(l_timeIntervalOpening)/60 << "h" << abs(l_timeIntervalOpening)%60 << "m" <<std::endl;
        std::cout << "Museum is open! " <<std::endl;
        m_printMuseumClosing = false;
    }
    response->is_ok = true;
}

void TimeComponent::StopTourTimer([[maybe_unused]]const std::shared_ptr<time_interfaces::srv::StopTourTimer::Request> request,
             std::shared_ptr<time_interfaces::srv::StopTourTimer::Response>      response) 
{
    m_timerMutex.lock();
    m_stopped = true;
    m_timerMutex.unlock();
    RCLCPP_INFO_STREAM(m_node->get_logger(), "TimeComponent::StopTourTimer ");
    if(m_timerTask == true)
    {
        if (m_threadTimer.joinable()) {
            m_threadTimer.join();
            RCLCPP_INFO_STREAM(m_node->get_logger(), "Timer task joined ");
        }
    }
    else
    {   
        RCLCPP_INFO_STREAM(m_node->get_logger(), "Timer task not running ");
    }
    response->is_ok = true;
}

void TimeComponent::publisher(std::string text)
{
    std_msgs::msg::String msg;
    msg.data = text;
    m_publisher->publish(msg);
}

std::string TimeComponent::getCurrentTime()
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&now_time);
    std::ostringstream oss;
    oss << std::put_time(now_tm, "%H:%M");
    std::string time = oss.str();
    std::cout << "Current time: " << time << std::endl;
    return time;
}

void TimeComponent::timerTask()
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "Start Timer");
    m_timerMutex.lock();
    m_timerTask = true;
    m_stopped = false;
    bool l_stopped = m_stopped;
    m_timerMutex.unlock();
    m_mutex.lock();
    int l_maxTime = m_maxTime;
    int l_warningTime = m_warningTime;
    m_mutex.unlock();
    int l_secondsPassed = 0;
    writeInBB(WARNING_BB_STRING, 0);
    writeInBB(MAX_BB_STRING, 0);
    writeInBB(DURATION_BB_STRING, 0);
    publisher("Tour started!");
    while(!l_stopped){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        l_secondsPassed++;
        if(l_secondsPassed == l_maxTime*60)
        {
            std::cout << "Time exceeded" << std::endl;
            writeInBB(MAX_BB_STRING, 1);
            publisher("Tour maximum time exceeded!");
        }
        else if(l_secondsPassed == l_warningTime*60)
        {
            std::cout << "Time warning" << std::endl;
            writeInBB(WARNING_BB_STRING, 1);
            publisher("Tour warning time exceeded!");
        }
        m_timerMutex.lock();
        l_stopped = m_stopped;
        m_timerMutex.unlock();
        if(l_secondsPassed % 60 == 0)
        {
            RCLCPP_INFO_STREAM(m_node->get_logger(), "Time passed: " << l_secondsPassed / 60 << " minutes");
            writeInBB(DURATION_BB_STRING, l_secondsPassed / 60);
        }
        
    }
    m_timerMutex.lock();
    m_timerTask = false;
    m_stopped = false;
    m_timerMutex.unlock();
    writeInBB(WARNING_BB_STRING, 0);
    writeInBB(MAX_BB_STRING, 0);
    writeInBB(DURATION_BB_STRING, 0);
    publisher("Tour ended!");
    RCLCPP_INFO_STREAM(m_node->get_logger(), "End Timer ");
}


bool TimeComponent::getParamsFromFile(std::ifstream& file)
{
    std::string line;
    std::string currentSection;
    std::unordered_map<std::string, std::unordered_map<std::string, std::string>> config;
    while (std::getline(file, line)) {
        // Trim the line (removing leading and trailing whitespace)
        line.erase(0, line.find_first_not_of(" \t\n\r"));
        line.erase(line.find_last_not_of(" \t\n\r") + 1);
        // Skip comments and empty lines
        if (line.empty() || line[0] == ';' || line[0] == '#') {
            continue;
        }
        // Check if the line is a section header
        if (line.front() == '[' && line.back() == ']') {
            currentSection = line.substr(1, line.size() - 2);
        } else {
            // Split the line into key and value
            std::istringstream lineStream(line);
            std::string key, value;
            if (std::getline(lineStream, key, '=')) {
                if (std::getline(lineStream >> std::ws, value)) {
                    config[currentSection][key] = value;
                }
            }
        }
    }
    // Accessing the values
    if(config.find("TIMER DURATION") != config.end())
    {
        std::string max_time, warning_time;
        if(!getValue(config, "TIMER DURATION", "maximum_duration", max_time) || !getValue(config, "TIMER DURATION", "warning_duration", warning_time))
        {
            return false;
        }
        m_mutex.lock();
        m_warningTime = std::stoi(warning_time);
        m_maxTime = std::stoi(max_time);
        m_mutex.unlock();
    }
    else
    {
        std::cerr << "Error: TIMER DURATION section not found in config file" << std::endl;
        return false;
    }
    if(config.find("MUSEUM HOURS") != config.end())
    {
        std::string opening_time, closing_time;
        if(!getValue(config, "MUSEUM HOURS", "opening_time", opening_time) || !getValue(config, "MUSEUM HOURS", "closing_time", closing_time))
        {
            return false;
        }
        m_mutex.lock();
        m_openingTime = opening_time;
        m_closingTime = closing_time;
        m_mutex.unlock();
    }
    else
    {
        std::cerr << "Error: MUSEUM HOURS section not found in config file" << std::endl;
        return false;
    }
    return true;
}

bool TimeComponent::loadConfigFile(){
    std::ifstream file(m_configPath);
    if (!file.is_open()) {
        std::cerr << "Unable to open file";
        return false;
    }
    if(!getParamsFromFile(file))
    {
        return false;
    }
    file.close();
    return true;
}

bool TimeComponent::getValue(std::unordered_map<std::string, std::unordered_map<std::string, std::string>> config, const std::string section, const std::string key, std::string& value)
{
    if(config[section].find(key) != config[section].end())
    {
        value = config[section][key];
        std::cout << key << ": " << value << std::endl;
        return true;
    }
    else
    {
        std::cerr << "Error: '" << key << "' not found in '" << section << "' section" << std::endl;
        return false;
    }
}

int TimeComponent::getTimeInterval(const std::string timeStamp1, const std::string timeStamp2)
{
    std::tm tm1 = {};
    std::tm tm2 = {};
    std::istringstream ss1(timeStamp1);
    std::istringstream ss2(timeStamp2);
    ss1 >> std::get_time(&tm1, "%H:%M");
    ss2 >> std::get_time(&tm2, "%H:%M");
    std::time_t time1 = std::mktime(&tm1);
    std::time_t time2 = std::mktime(&tm2);
    return std::difftime(time2, time1) / 60; // in minutes
}

bool TimeComponent::writeInBB(std::string key, int value)
{
    //calls the SetInt service 
    auto setIntClientNode = rclcpp::Node::make_shared("BlackboardComponentSetIntNode");
    std::shared_ptr<rclcpp::Client<blackboard_interfaces::srv::SetIntBlackboard>> setIntClient = 
    setIntClientNode->create_client<blackboard_interfaces::srv::SetIntBlackboard>("/BlackboardComponent/SetInt"); 
    auto setIntRequest = std::make_shared<blackboard_interfaces::srv::SetIntBlackboard::Request>();
    setIntRequest->field_name = key;
    setIntRequest->value = value;
    bool wait_succeded{true};
    int retries = 0;
    while (!setIntClient->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service '/BlackboardComponent/SetInt'. Exiting.");
            wait_succeded = false;
            return false;
        }
        retries++;
        if(retries == SERVICE_TIMEOUT) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service '/BlackboardComponent/SetInt'.");
            wait_succeded = false;
            return false;
        }
    }
    if (!wait_succeded) {
        return false;
    }
    auto setIntResult = setIntClient->async_send_request(setIntRequest);
    auto futureSetIntResult = rclcpp::spin_until_future_complete(setIntClientNode, setIntResult);
    auto setIntFutureResult = setIntResult.get();
    if (setIntFutureResult->is_ok == true) {
        return true;
    }
    return false;
}