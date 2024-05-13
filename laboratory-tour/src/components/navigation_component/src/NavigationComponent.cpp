/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "NavigationComponent.h"

#include <navigation_interfaces/msg/navigation_status.hpp>


bool NavigationComponent::start(int argc, char*argv[])
{

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("NavigationComponentNode");
    m_goToPoiByNameService = m_node->create_service<navigation_interfaces::srv::GoToPoiByName>("/NavigationComponent/GoToPoiByName",  
                                                                                std::bind(&NavigationComponent::GoToPoiByName,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getNavigationStatusService = m_node->create_service<navigation_interfaces::srv::GetNavigationStatus>("/NavigationComponent/GetNavigationStatus",  
                                                                                std::bind(&NavigationComponent::GetNavigationStatus,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_stopNavigationService = m_node->create_service<navigation_interfaces::srv::StopNavigation>("/NavigationComponent/StopNavigation",  
                                                                                std::bind(&NavigationComponent::StopNavigation,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_checkNearToPoiService = m_node->create_service<navigation_interfaces::srv::CheckNearToPoi>("/NavigationComponent/CheckNearToPoi",  
                                                                                std::bind(&NavigationComponent::CheckNearToPoi,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));

    m_turnBackService = m_node->create_service<navigation_interfaces::srv::TurnBack>("/NavigationComponent/TurnBack",  
                                                                                std::bind(&NavigationComponent::TurnBack,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));

    RCLCPP_DEBUG(m_node->get_logger(), "NavigationComponent::start");
    std::cout << "NavigationComponent::start";        
    return true;

}


bool NavigationComponent::ConfigureYARP(yarp::os::ResourceFinder &rf)
{
    bool okNav = rf.check("NAVIGATION2D-CLIENT");
    std::string device = "navigation2D_nwc_yarp";
    std::string local = "/NavigationComponentNode/navClient";
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
            local = "/NavigationComponentNode" + nav_config.find("local-suffix").asString();
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


bool NavigationComponent::close()
{
    rclcpp::shutdown();  
    return true;
}

void NavigationComponent::spin()
{
    rclcpp::spin(m_node);  
}


void NavigationComponent::GetNavigationStatus([[maybe_unused]] const std::shared_ptr<navigation_interfaces::srv::GetNavigationStatus::Request> request,
             std::shared_ptr<navigation_interfaces::srv::GetNavigationStatus::Response>      response) 
{
    std::string target;
    yarp::dev::Nav2D::NavigationStatusEnum status;
    auto msg = navigation_interfaces::msg::NavigationStatus();
    if (!m_iNav2D->getNameOfCurrentTarget(target))
    {
        response->is_ok = false;
        response->error_msg = "failed to get target";
    }
    else if(!m_iNav2D->getNavigationStatus(status))
    {
        response->is_ok = false;
        response->error_msg = "failed to get status";
    } else 
    {
        response->is_ok = true;
        response->current_goal = target;
        switch (status){
            case yarp::dev::Nav2D::navigation_status_idle:
                response->status.status = msg.NAVIGATION_STATUS_IDLE;
                break;
            case yarp::dev::Nav2D::navigation_status_preparing_before_move:
                response->status.status = msg.NAVIGATION_STATUS_PREPARING_BEFORE_MOVE;
                break;
            case yarp::dev::Nav2D::navigation_status_moving:
                response->status.status = msg.NAVIGATION_STATUS_MOVING;
                break;
            case yarp::dev::Nav2D::navigation_status_waiting_obstacle:
                response->status.status = msg.NAVIGATION_STATUS_WAITING_OBSTACLE;
                break;
            case yarp::dev::Nav2D::navigation_status_goal_reached:
                response->status.status = msg.NAVIGATION_STATUS_GOAL_REACHED;
                break;
            case yarp::dev::Nav2D::navigation_status_aborted:
                response->status.status = msg.NAVIGATION_STATUS_ABORTED;
                break;
            case yarp::dev::Nav2D::navigation_status_failing:
                response->status.status = msg.NAVIGATION_STATUS_FAILING;
                break;
            case yarp::dev::Nav2D::navigation_status_paused:
                response->status.status = msg.NAVIGATION_STATUS_PAUSED;
                break;
            case yarp::dev::Nav2D::navigation_status_thinking:
                response->status.status = msg.NAVIGATION_STATUS_THINKING;
                break;
            case yarp::dev::Nav2D::navigation_status_error:
                response->status.status = msg.NAVIGATION_STATUS_ERROR;
                break;
            default:
                response->status.status = msg.NAVIGATION_STATUS_ERROR;
                break;
        }
    }
}


void NavigationComponent::GoToPoiByName(const std::shared_ptr<navigation_interfaces::srv::GoToPoiByName::Request> request,
             std::shared_ptr<navigation_interfaces::srv::GoToPoiByName::Response>      response) 
{
    if (request->poi_name == "")
    {
        response->is_ok = false;
        response->error_msg = "empty poi";
    } else if (!m_iNav2D->gotoTargetByLocationName(request->poi_name))
    {
        response->is_ok = false;
        response->error_msg = "failed to send goal";
    } else 
    {
        response->is_ok = true;
    }
}


void NavigationComponent::StopNavigation([[maybe_unused]] const std::shared_ptr<navigation_interfaces::srv::StopNavigation::Request> request,
             std::shared_ptr<navigation_interfaces::srv::StopNavigation::Response>      response) 
{
    if(!m_iNav2D->stopNavigation())
    {
        response->is_ok = false;
        response->error_msg = "failed to stop navigation";
    } else 
    {
        response->is_ok = true;
    }
}



void NavigationComponent::CheckNearToPoi(const std::shared_ptr<navigation_interfaces::srv::CheckNearToPoi::Request> request,
             std::shared_ptr<navigation_interfaces::srv::CheckNearToPoi::Response>      response) 
{
    if (request->poi_name == "")
    {
        response->is_ok = false;
        response->error_msg = "empty poi";
    } else if (!m_iNav2D->checkNearToLocation(request->poi_name, request->distance))
    // } else if (!m_iNav2D->checkNearToLocation(request->poi_name, request->distance, request->angle))
    {
        // std::cout << "Request Angle " << request->angle << std::cout;
        
        response->is_ok = true;
        response->is_near = false;
        response->error_msg = "failed to check if nearby";
    } else 
    {
        response->is_ok = true;
        response->is_near = true;
    }
}


void NavigationComponent::TurnBack([[maybe_unused]] const std::shared_ptr<navigation_interfaces::srv::TurnBack::Request> request,
             std::shared_ptr<navigation_interfaces::srv::TurnBack::Response>      response) 
{
    if(!m_iNav2D->stopNavigation())
    {
        response->is_ok = false;
        response->error_msg = "failed to stop navigation";
    } else 
    {
        response->is_ok = true;
    }
    if(!m_iNav2D->gotoTargetByRelativeLocation(0, 0, 180))
    {
        response->is_ok = false;
        response->error_msg = "failed to turn back";
    } else 
    {
        response->is_ok = true;
    }
}