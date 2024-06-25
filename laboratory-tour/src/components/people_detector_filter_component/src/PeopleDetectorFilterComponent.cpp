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
    m_publisher = m_node->create_publisher<people_detector_filter_interfaces::msg::FilterStatus>("/PeopleDetectorFilterComponent/detection", 10);
    m_filteredPublisher = m_node->create_publisher<std_msgs::msg::Bool>("/PeopleDetectorFilterComponent/filtered_detection", 10);
    m_timer = m_node->create_wall_timer(std::chrono::seconds(1), std::bind(&PeopleDetectorFilterComponent::publisher, this));
    m_timer_filtered = m_node->create_wall_timer(std::chrono::seconds(1), std::bind(&PeopleDetectorFilterComponent::publisher_filtered, this));
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
        // std::cout << "Neutral set, turning " << std::endl;
        m_taskMutex.lock();
        m_neutralTurningDetected = true;
        m_taskMutex.unlock();
    }
}

bool PeopleDetectorFilterComponent::getNavigationStatus(yarp::dev::Nav2D::NavigationStatusEnum& status)
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
            m_neutralStatusDetected = true;
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
            m_neutralStatusDetected = true;
            m_taskMutex.unlock();
            break;
        case yarp::dev::Nav2D::navigation_status_aborted:
            std::cout << "Navigation status aborted " << std::endl;
            std::cout << "Neutral set, status aborted" << std::endl;
            m_taskMutex.lock();
            m_neutralStatusDetected = true;
            m_taskMutex.unlock();
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

bool PeopleDetectorFilterComponent::getTurningBackStatus(std::string& turning_back_status)
{
    //calls the GetString service 
    auto getStringClientNode = rclcpp::Node::make_shared("BlackboardComponentGetStringNode");
    std::shared_ptr<rclcpp::Client<blackboard_interfaces::srv::GetStringBlackboard>> getStringClient = 
    getStringClientNode->create_client<blackboard_interfaces::srv::GetStringBlackboard>("/BlackboardComponent/GetString");
    auto getStringRequest = std::make_shared<blackboard_interfaces::srv::GetStringBlackboard::Request>();
    getStringRequest->field_name = TURNING_BACK_BB_STR;
    bool wait_succeded{true};
    int retries = 0;
    while (!getStringClient->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service '/BlackboardComponent/GetString'. Exiting.");
            wait_succeded = false;
            return false;
        }
        retries++;
        if(retries == SERVICE_TIMEOUT) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service '/BlackboardComponent/GetString'.");
            wait_succeded = false;
            return false;
        }
    }
    if (!wait_succeded) {
        return false;
    }
    auto getStringResult = getStringClient->async_send_request(getStringRequest);
    auto futureGetStringResult = rclcpp::spin_until_future_complete(getStringClientNode, getStringResult);
    auto getStringFutureResult = getStringResult.get();
    if (getStringFutureResult->is_ok == true) { // 0 is getString
        turning_back_status = getStringFutureResult->value;
        return true;
    }
    return false;
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
    OutputStatus l_outputStatus;
    OutputStatus l_oldStatus;
    int32_t l_filterCounter{0};
    do{
        bool l_neutralDetected = false;
        getNavigationStatus(status);
        m_taskMutex.lock();
        l_filterCounter = m_filterCounter;
        l_oldStatus = m_oldStatus;
        if (m_neutralStatusDetected){
            l_neutralDetected = true;
            m_neutralStatusDetected = false;
            l_outputStatus = OutputStatus::NEUTRAL_STATUS_NEUTRAL_NAV_STATUS;
            std::cout << "Neutral detected from navigation status" << std::endl;
        }
        if (m_neutralTurningDetected){
            l_neutralDetected = true;
            m_neutralTurningDetected = false;
            l_outputStatus = OutputStatus::NEUTRAL_STATUS_TURNING;
            std::cout << "Neutral detected, robot is turning" << std::endl;
        }
        m_taskMutex.unlock();

        std::string turning_back_status;
        getTurningBackStatus(turning_back_status);

        if(turning_back_status == TURNING_BACK_STATUS_TURNING)
        {
            std::cout << "Neutral detected, robot is turing back" << std::endl;
            l_outputStatus = OutputStatus::NEUTRAL_STATUS_TURNING_BACK;
        }
        else if(turning_back_status == TURNING_BACK_STATUS_TURNED)
        {
            std::cout << "Neutral detected, robot is turned back" << std::endl;
            l_outputStatus = OutputStatus::NEUTRAL_STATUS_TURN_BACK_REACHED;
        }
        else if(l_neutralDetected)
        {
            // If neutral state detected (robot is turning or navigation status is goal reached or idle)
            // Check if it was turning back to check for people
            l_filterCounter = 0;
        }
        else if(m_peopleDetectorStatus)
        {
            l_outputStatus = OutputStatus::TRUE_STATUS;
        }
        else
        {
            l_outputStatus = OutputStatus::FALSE_STATUS;
        }

        if(l_oldStatus != l_outputStatus)
        {
            if((l_oldStatus == OutputStatus::NEUTRAL_STATUS_TURNING || l_oldStatus == OutputStatus::NEUTRAL_STATUS_NEUTRAL_NAV_STATUS ))
            {
                if(l_outputStatus != OutputStatus::NEUTRAL_STATUS_TURNING_BACK && l_outputStatus != OutputStatus::NEUTRAL_STATUS_TURN_BACK_REACHED)
                {  
                    if(l_filterCounter < m_filterTimeout)
                    {
                        RCLCPP_INFO_STREAM(m_node->get_logger(), "Forcing neutral state until timeout, counter: "<< l_filterCounter);
                        l_outputStatus = l_oldStatus;
                    }
                }
            }
        }
        l_filterCounter++;
        RCLCPP_INFO_STREAM(m_node->get_logger(), "Output status: "<< l_outputStatus);

        m_taskMutex.lock();
        m_filterCounter = l_filterCounter;
        m_outputStatus = l_outputStatus;
        m_oldStatus = l_outputStatus;
        l_stop = m_stopTask;
        m_taskMutex.unlock();

        std::this_thread::sleep_for(std::chrono::seconds(1)); //delete
    }while(!l_stop);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "END_COMPUTE_THREAD");
    m_computeTask = false;
}


void PeopleDetectorFilterComponent::publisher()
{
    people_detector_filter_interfaces::msg::FilterStatus msg;
    msg.status = m_outputStatus;
    m_publisher->publish(msg);
}

void PeopleDetectorFilterComponent::publisher_filtered()
{
    OutputStatus l_filteredStatus = m_outputStatus;
    std_msgs::msg::Bool msg;

    if(m_outputStatus == NEUTRAL_STATUS_TURNING || m_outputStatus == NEUTRAL_STATUS_NEUTRAL_NAV_STATUS)
    {
        l_filteredStatus = OutputStatus::TRUE_STATUS;
    }
    else if(m_outputStatus == NEUTRAL_STATUS_TURNING_BACK)
    {
        l_filteredStatus = OutputStatus::FALSE_STATUS;
    }
    else if(m_outputStatus == NEUTRAL_STATUS_TURN_BACK_REACHED)
    {
        if(m_peopleDetectorStatus)
            l_filteredStatus = OutputStatus::TRUE_STATUS;
        else
            l_filteredStatus = OutputStatus::FALSE_STATUS;
    }
    if( l_filteredStatus == OutputStatus::TRUE_STATUS)
        msg.data = true;
    else
        msg.data = false;
    m_filteredPublisher->publish(msg);
}