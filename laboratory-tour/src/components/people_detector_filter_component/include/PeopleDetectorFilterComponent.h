/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>
#include <people_detector_filter_interfaces/srv/set_filter_timeout.hpp>
#include <people_detector_filter_interfaces/srv/get_filter_timeout.hpp>
#include <people_detector_filter_interfaces/msg/filter_status.hpp>
#include <blackboard_interfaces/srv/get_string_blackboard.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#define SERVICE_TIMEOUT 1
#define TURNING_BACK_BB_STR             "turnBackState"
#define TURNING_BACK_STATUS_TURNING     "turning"
#define TURNING_BACK_STATUS_TURNED      "turned"
#define TURNING_BACK_STATUS_NOT_TURNED  "not_turning"

enum OutputStatus
{
    TRUE_STATUS = people_detector_filter_interfaces::msg::FilterStatus::FILTER_STATUS_TRUE,
    FALSE_STATUS = people_detector_filter_interfaces::msg::FilterStatus::FILTER_STATUS_FALSE,
    NEUTRAL_STATUS_TURN_BACK_REACHED = people_detector_filter_interfaces::msg::FilterStatus::FILTER_STATUS_NEUTRAL_TURN_BACK_REACHED,
    NEUTRAL_STATUS_TURNING_BACK = people_detector_filter_interfaces::msg::FilterStatus::FILTER_STATUS_NEUTRAL_TURNING_BACK,
    NEUTRAL_STATUS_TURNING = people_detector_filter_interfaces::msg::FilterStatus::FILTER_STATUS_NEUTRAL_TURNING,
    NEUTRAL_STATUS_NEUTRAL_NAV_STATUS = people_detector_filter_interfaces::msg::FilterStatus::FILTER_STATUS_NEUTRAL_NAV_STATUS

};

class PeopleDetectorFilterComponent 
{
public:
    PeopleDetectorFilterComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    bool ConfigureYARP(yarp::os::ResourceFinder &rf);
    void topic_callback_people_detector(const std_msgs::msg::Bool::SharedPtr msg);
    void topic_callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publisher();
    void publisher_filtered();
    bool startComputeOutput();
    void stopComputeOutput();
    void computeOutputTask();
    bool getNavigationStatus(yarp::dev::Nav2D::NavigationStatusEnum& status);
    bool getTurningBackStatus(std::string& turning_back_status);

    void SetFilterTimeout( const std::shared_ptr<people_detector_filter_interfaces::srv::SetFilterTimeout::Request> request,
                std::shared_ptr<people_detector_filter_interfaces::srv::SetFilterTimeout::Response>      response);
    void GetFilterTimeout([[maybe_unused]] const std::shared_ptr<people_detector_filter_interfaces::srv::GetFilterTimeout::Request> request,
                std::shared_ptr<people_detector_filter_interfaces::srv::GetFilterTimeout::Response>      response);
    

private:
    yarp::dev::PolyDriver m_nav2DPoly;
    yarp::dev::Nav2D::INavigation2D *m_iNav2D{nullptr};
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<people_detector_filter_interfaces::srv::SetFilterTimeout>::SharedPtr m_setFilterTimeoutService;
    rclcpp::Service<people_detector_filter_interfaces::srv::GetFilterTimeout>::SharedPtr m_getFilterTimeoutService;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscriptionPeopleDetector;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_subscriptionOdometry;
    rclcpp::Publisher<people_detector_filter_interfaces::msg::FilterStatus>::SharedPtr m_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_filteredPublisher;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::TimerBase::SharedPtr m_timer_filtered;
    std::mutex m_mutex;
    std::mutex m_taskMutex;
    int32_t m_filterTimeout{15};
    int32_t m_filterCounter{0};
    int32_t m_turningStepCount{0};
    bool m_peopleDetectorStatus{true};
    bool m_neutralTurningDetected{false};
    bool m_neutralStatusDetected{false};
    bool m_computeTask{false};
    bool m_stopTask{false};
    std::thread computeThread;
    yarp::dev::Nav2D::NavigationStatusEnum m_navigationStatus;
    OutputStatus m_outputStatus{OutputStatus::NEUTRAL_STATUS_NEUTRAL_NAV_STATUS};
    OutputStatus m_oldStatus{OutputStatus::NEUTRAL_STATUS_NEUTRAL_NAV_STATUS};
    
};
