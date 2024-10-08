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
#include <navigation_interfaces_dummy/msg/navigation_status.hpp>
#include <navigation_interfaces_dummy/action/go_to_poi.hpp>


class NavigationComponent 
{
public:
    NavigationComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    bool ConfigureYARP(yarp::os::ResourceFinder &rf);

    void Execute(const std::shared_ptr<navigation_interfaces_dummy::action::GoToPoi::Goal> goal);
    void ExecuteCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_interfaces_dummy::action::GoToPoi>> goalHandle);
    
private:
    yarp::dev::PolyDriver m_nav2DPoly;
    yarp::dev::Nav2D::INavigation2D *m_iNav2D{nullptr};
    rclcpp::Node::SharedPtr m_node;

    std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_interfaces_dummy::action::GoToPoi>> m_activeGoal;
    std::mutex m_goalMutex;

	rclcpp_action::Server<navigation_interfaces_dummy::action::GoToPoi>::SharedPtr m_actionServer;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const navigation_interfaces_dummy::action::GoToPoi::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_interfaces_dummy::action::GoToPoi>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_interfaces_dummy::action::GoToPoi>> goal_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_interfaces_dummy::action::GoToPoi>> goal_handle);
    navigation_interfaces_dummy::msg::NavigationStatus convertStatus(yarp::dev::Nav2D::NavigationStatusEnum status);
    std::mutex m_mutex;
    int32_t m_currentPoi;
};
