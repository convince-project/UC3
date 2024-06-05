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
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>

enum OutputStatus
{
    TRUE_STATUS = people_detector_filter_interfaces::msg::FilterStatus::FILTER_STATUS_TRUE,
    FALSE_STATUS = people_detector_filter_interfaces::msg::FilterStatus::FILTER_STATUS_FALSE,
    NEUTRAL_STATUS = people_detector_filter_interfaces::msg::FilterStatus::FILTER_STATUS_NEUTRAL,
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
    void SetFilterTimeout( const std::shared_ptr<people_detector_filter_interfaces::srv::SetFilterTimeout::Request> request,
                std::shared_ptr<people_detector_filter_interfaces::srv::SetFilterTimeout::Response>      response);
    void GetFilterTimeout([[maybe_unused]] const std::shared_ptr<people_detector_filter_interfaces::srv::GetFilterTimeout::Request> request,
                std::shared_ptr<people_detector_filter_interfaces::srv::GetFilterTimeout::Response>      response);
    bool startComputeOutput();
    void stopComputeOutput();
    void computeOutputTask();
    bool getNavigationStatus(yarp::dev::Nav2D::NavigationStatusEnum status);

private:
    yarp::dev::PolyDriver m_nav2DPoly;
    yarp::dev::Nav2D::INavigation2D *m_iNav2D{nullptr};
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<people_detector_filter_interfaces::srv::SetFilterTimeout>::SharedPtr m_setFilterTimeoutService;
    rclcpp::Service<people_detector_filter_interfaces::srv::GetFilterTimeout>::SharedPtr m_getFilterTimeoutService;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscriptionPeopleDetector;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_subscriptionOdometry;
    rclcpp::Publisher<people_detector_filter_interfaces::msg::FilterStatus>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::mutex m_mutex;
    std::mutex m_taskMutex;
    int32_t m_filterTimeout{15};
    int32_t m_filterCounter{0};
    int32_t m_turningStepCount{0};
    bool m_peopleDetectorStatus{true};
    bool m_neutralDetected{false};
    bool m_computeTask{false};
    bool m_stopTask{false};
    std::thread computeThread;
    yarp::dev::Nav2D::NavigationStatusEnum m_navigationStatus;
    OutputStatus m_outputStatus{OutputStatus::NEUTRAL_STATUS};
    OutputStatus m_oldStatus{OutputStatus::NEUTRAL_STATUS};
    
};
