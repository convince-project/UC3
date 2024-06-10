/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "PeopleDetectorFilterComponent.h"


bool PeopleDetectorFilterComponent::start(int argc, char*argv[])
{

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("PeopleDetectorFilterComponentNode");
    m_setFilterTimeoutService = m_node->create_service<people_detector_filter_interfaces::srv::SetFilterTimeout>("/PeopleDetectorFilterComponent/SetFilterTimeout",  
                                                                                std::bind(&PeopleDetectorFilterComponent::SetFilterTimeout,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getFilterTimeoutService = m_node->create_service<people_detector_filter_interfaces::srv::GetFilterTimeout>("/PeopleDetectorFilterComponent/GetFilterTimeout",  
                                                                                std::bind(&PeopleDetectorFilterComponent::GetFilterTimeout,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));

    RCLCPP_DEBUG(m_node->get_logger(), "PeopleDetectorFilterComponent::start");
    m_subscriptionPeopleDetector = m_node->create_subscription<std_msgs::msg::Bool>(
		"is_followed_timeout", 10, std::bind(&PeopleDetectorFilterComponent::topic_callback_people_detector, this, std::placeholders::_1));
    m_subscriptionOdometry = m_node->create_subscription<nav_msgs::msg::Odometry>(
		"odometry", 10, std::bind(&PeopleDetectorFilterComponent::topic_callback_odometry, this, std::placeholders::_1));
    m_publisher = m_node->create_publisher<people_detector_filter_interfaces::msg::FilterStatus>("/PeopleDetectorFilterComponent/filtered_detection", 10);
    m_timer = m_node->create_wall_timer(std::chrono::seconds(1), std::bind(&PeopleDetectorFilterComponent::publisher, this));
    std::cout << "PeopleDetectorFilterComponent::start";        
    return true;

}


bool PeopleDetectorFilterComponent::ConfigureYARP(yarp::os::ResourceFinder &rf)
{
    bool okNav = rf.check("NAVIGATION2D-CLIENT");
    std::string device = "navigation2D_nwc_yarp";
    std::string local = "/PeopleDetectorFilterComponentNode/navClient";
    std::string navServer = "/navigation2D_nws_yarp";
    std::string mapServer = "/map2D_nws_yarp";
    std::string locServer = "/localization2D_nws_yarp";
    if (okNav)
    {
        yarp::os::Searchable &nav_config = rf.findGroup("NAVIGATION2D-CLIENT");
        if (nav_config.check("device"))
        {
            device = nav_config.find("device").asString();
        }
        if (nav_config.check("local-suffix"))
        {
            local = "/PeopleDetectorFilterComponentNode" + nav_config.find("local-suffix").asString();
        }
        if (nav_config.check("navigation_server"))
        {
            navServer = nav_config.find("navigation_server").asString();
        }
        if (nav_config.check("map_locations_server"))
        {
            mapServer = nav_config.find("map_locations_server").asString();
        }
        if (nav_config.check("localization_server"))
        {
            locServer = nav_config.find("localization_server").asString();
        }
    }

    yarp::os::Property nav2DProp;
    nav2DProp.put("device", device);
    nav2DProp.put("local", local);
    nav2DProp.put("navigation_server", navServer);
    nav2DProp.put("map_locations_server", mapServer);
    nav2DProp.put("localization_server", locServer);
    nav2DProp.put("period", 5);

    m_nav2DPoly.open(nav2DProp);
    if (!m_nav2DPoly.isValid())
    {
        yError() << "Error opening Nav Client PolyDriver. Check parameters";
        return false;
    }
    m_nav2DPoly.view(m_iNav2D);
    if (!m_iNav2D)
    {
        yError() << "Error opening iNav2D interface. Device not available";
        return false;
    }
    return true;
}


bool PeopleDetectorFilterComponent::close()
{
    rclcpp::shutdown();
    stopComputeOutput();  
    return true;
}

void PeopleDetectorFilterComponent::spin()
{
    rclcpp::spin(m_node);  
}


void PeopleDetectorFilterComponent::SetFilterTimeout(const std::shared_ptr<people_detector_filter_interfaces::srv::SetFilterTimeout::Request> request,
             std::shared_ptr<people_detector_filter_interfaces::srv::SetFilterTimeout::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "PeopleDetectorFilterComponent::SetFilterTimeout value: " << request->timeout);
    m_filterTimeout = request->timeout;
    response->is_ok = true;

}

void PeopleDetectorFilterComponent::GetFilterTimeout([[maybe_unused]]const std::shared_ptr<people_detector_filter_interfaces::srv::GetFilterTimeout::Request> request,
             std::shared_ptr<people_detector_filter_interfaces::srv::GetFilterTimeout::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "PeopleDetectorFilterComponent::GetFilterTimeout value: " << m_filterTimeout);
    response->timeout = m_filterTimeout;
    response->is_ok = true;

}

void PeopleDetectorFilterComponent::topic_callback_people_detector(const std_msgs::msg::Bool::SharedPtr msg) {
	m_peopleDetectorStatus = msg->data;
}

void PeopleDetectorFilterComponent::topic_callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if(fabs(msg->twist.twist.angular.z) > 0.2)
    {
        m_turningStepCount++;
    }
    else{
        m_turningStepCount = 0;
    }
    if(m_turningStepCount > 2)
    {
        std::cout << "Neutral set, turning " << std::endl;
        m_taskMutex.lock();
        m_neutralDetected = true;
        m_taskMutex.unlock();
    }
}

bool PeopleDetectorFilterComponent::startComputeOutput()
{
    if(!m_computeTask){
        m_computeTask = true;
        computeThread = std::thread(&PeopleDetectorFilterComponent::computeOutputTask, this);
        return true;
    }
    return false;
}

void PeopleDetectorFilterComponent::stopComputeOutput()
{
    if(m_computeTask){
        m_taskMutex.lock();
        m_stopTask = true;
        m_taskMutex.unlock();
        if (computeThread.joinable()) {
            computeThread.join();
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "STOP_COMPUTE_THREAD");
    }
}

void PeopleDetectorFilterComponent::computeOutputTask()
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "NEW_COMPUTE_THREAD");
    bool l_stop;
    yarp::dev::Nav2D::NavigationStatusEnum status;
    do{
        bool l_neutralDetected = false;
        getNavigationStatus(status);
        m_taskMutex.lock();
        if (m_neutralDetected){
            l_neutralDetected = m_neutralDetected;
            m_neutralDetected = false;
            std::cout << "Neutral detected " << std::endl;
        }
        m_taskMutex.unlock();

        if(l_neutralDetected)
        {
            m_filterCounter = 0;
            m_outputStatus = OutputStatus::NEUTRAL_STATUS;
        }
        else if(m_peopleDetectorStatus)
        {
            m_outputStatus = OutputStatus::TRUE_STATUS;
        }
        else
        {
            m_outputStatus = OutputStatus::FALSE_STATUS;
        }
        if(m_oldStatus != m_outputStatus)
        {
            if(m_oldStatus == OutputStatus::NEUTRAL_STATUS && m_filterCounter < m_filterTimeout)
            {
                RCLCPP_INFO_STREAM(m_node->get_logger(), "Forcing neutral state until timeout, counter: "<< m_filterCounter);
                m_outputStatus = OutputStatus::NEUTRAL_STATUS;
            }
        }
        m_filterCounter++;
        RCLCPP_INFO_STREAM(m_node->get_logger(), "Output status: "<< m_outputStatus);
        m_oldStatus = m_outputStatus;
        m_taskMutex.lock();
        l_stop = m_stopTask;
        m_taskMutex.unlock();
        std::this_thread::sleep_for(std::chrono::seconds(1)); //delete
    }while(!l_stop);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "END_COMPUTE_THREAD");
    m_computeTask = false;
}

bool PeopleDetectorFilterComponent::getNavigationStatus(yarp::dev::Nav2D::NavigationStatusEnum status)
{
    if(!m_iNav2D->getNavigationStatus(status))
    {
        return false;
    } 
    switch (status){
        case yarp::dev::Nav2D::navigation_status_idle:
            std::cout << "Navigation status IDLE " << std::endl;
            std::cout << "Neutral set, status idle" << std::endl;
            m_taskMutex.lock();
            m_neutralDetected = true;
            m_taskMutex.unlock();
            break;
        case yarp::dev::Nav2D::navigation_status_preparing_before_move:
            std::cout << "Navigation status Preparing_before_move " << std::endl;
            break;
        case yarp::dev::Nav2D::navigation_status_moving:
            std::cout << "Navigation status moving " << std::endl;
            break;
        case yarp::dev::Nav2D::navigation_status_waiting_obstacle:
            std::cout << "Navigation status waiting_obstacle " << std::endl;
            break;
        case yarp::dev::Nav2D::navigation_status_goal_reached:
            std::cout << "Navigation status goal reached " << std::endl;
            std::cout << "Neutral set, goal reached" << std::endl;
            m_taskMutex.lock();
            m_neutralDetected = true;
            m_taskMutex.unlock();
            break;
        case yarp::dev::Nav2D::navigation_status_aborted:
            std::cout << "Navigation status aborted " << std::endl;
            break;
        case yarp::dev::Nav2D::navigation_status_failing:
            std::cout << "Navigation status failing " << std::endl;
            break;
        case yarp::dev::Nav2D::navigation_status_paused:
            std::cout << "Navigation status paused " << std::endl;
            break;
        case yarp::dev::Nav2D::navigation_status_thinking:
            std::cout << "Navigation status thinking " << std::endl;
            break;
        case yarp::dev::Nav2D::navigation_status_error:
            std::cout << "Navigation status error " << std::endl;
            break;
        default:
            std::cout << "Navigation status default error " << std::endl;
            break;
    }
    return true;
}

void PeopleDetectorFilterComponent::publisher()
{
    people_detector_filter_interfaces::msg::FilterStatus msg;
    msg.status = m_outputStatus;
    m_publisher->publish(msg);
}