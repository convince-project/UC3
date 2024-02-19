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
#include <navigation_interfaces/srv/go_to_poi_by_name.hpp>
#include <navigation_interfaces/srv/get_navigation_status.hpp>
#include <navigation_interfaces/srv/stop_navigation.hpp>
#include <navigation_interfaces/srv/check_near_to_poi.hpp>



class NavigationComponent 
{
public:
    NavigationComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    bool ConfigureYARP(yarp::os::ResourceFinder &rf);
    void GoToPoiByName( const std::shared_ptr<navigation_interfaces::srv::GoToPoiByName::Request> request,
                std::shared_ptr<navigation_interfaces::srv::GoToPoiByName::Response>      response);
    void GetNavigationStatus([[maybe_unused]] const std::shared_ptr<navigation_interfaces::srv::GetNavigationStatus::Request> request,
                std::shared_ptr<navigation_interfaces::srv::GetNavigationStatus::Response>      response);
    void StopNavigation( [[maybe_unused]] const std::shared_ptr<navigation_interfaces::srv::StopNavigation::Request> request,
            std::shared_ptr<navigation_interfaces::srv::StopNavigation::Response>      response);
    void CheckNearToPoi( [[maybe_unused]] const std::shared_ptr<navigation_interfaces::srv::CheckNearToPoi::Request> request,
            std::shared_ptr<navigation_interfaces::srv::CheckNearToPoi::Response>      response);

private:
    yarp::dev::PolyDriver m_nav2DPoly;
    yarp::dev::Nav2D::INavigation2D *m_iNav2D{nullptr};
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<navigation_interfaces::srv::GoToPoiByName>::SharedPtr m_goToPoiByNameService;
    rclcpp::Service<navigation_interfaces::srv::GetNavigationStatus>::SharedPtr m_getNavigationStatusService;
    rclcpp::Service<navigation_interfaces::srv::StopNavigation>::SharedPtr m_stopNavigationService;
    rclcpp::Service<navigation_interfaces::srv::CheckNearToPoi>::SharedPtr m_checkNearToPoiService;
    std::mutex m_mutex;
    int32_t m_currentPoi;
};
