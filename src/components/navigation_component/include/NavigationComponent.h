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
#include <rclcpp_action/rclcpp_action.hpp>
#include <navigation_interfaces/msg/navigation_status.hpp>
#include <navigation_interfaces/srv/go_to_poi_by_name.hpp>
#include <navigation_interfaces/srv/get_navigation_status.hpp>
#include <navigation_interfaces/srv/stop_navigation.hpp>
#include <navigation_interfaces/srv/check_near_to_poi.hpp>
#include <navigation_interfaces/srv/turn_back.hpp>
#include <navigation_interfaces/action/go_to_poi.hpp>


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
    void TurnBack( [[maybe_unused]] const std::shared_ptr<navigation_interfaces::srv::TurnBack::Request> request,
            std::shared_ptr<navigation_interfaces::srv::TurnBack::Response>      response);

    void Execute(const std::shared_ptr<navigation_interfaces::action::GoToPoi::Goal> goal);
    void ExecuteCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_interfaces::action::GoToPoi>> goalHandle);
    
private:
    yarp::dev::PolyDriver m_nav2DPoly;
    yarp::dev::Nav2D::INavigation2D *m_iNav2D{nullptr};
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<navigation_interfaces::srv::GoToPoiByName>::SharedPtr m_goToPoiByNameService;
    rclcpp::Service<navigation_interfaces::srv::GetNavigationStatus>::SharedPtr m_getNavigationStatusService;
    rclcpp::Service<navigation_interfaces::srv::StopNavigation>::SharedPtr m_stopNavigationService;
    rclcpp::Service<navigation_interfaces::srv::CheckNearToPoi>::SharedPtr m_checkNearToPoiService;
    rclcpp::Service<navigation_interfaces::srv::TurnBack>::SharedPtr m_turnBackService;

    std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_interfaces::action::GoToPoi>> m_activeGoal;
    std::mutex m_goalMutex;

	rclcpp_action::Server<navigation_interfaces::action::GoToPoi>::SharedPtr m_actionServer;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const navigation_interfaces::action::GoToPoi::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_interfaces::action::GoToPoi>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_interfaces::action::GoToPoi>> goal_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_interfaces::action::GoToPoi>> goal_handle);
    navigation_interfaces::msg::NavigationStatus convertStatus(yarp::dev::Nav2D::NavigationStatusEnum status);
    std::mutex m_mutex;
    int32_t m_currentPoi;
};
