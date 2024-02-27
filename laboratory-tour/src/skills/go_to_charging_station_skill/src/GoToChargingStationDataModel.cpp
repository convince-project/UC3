/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
#include "GoToChargingStationDataModel.h"

#include <QDebug>
#include <QTimer>
#include <QScxmlStateMachine>
#include <thread>


void GoToChargingStationDataModel::set_name(std::string name)
{
    m_name=name; 
}

bool GoToChargingStationDataModel::setup(const QVariantMap &initialDataValues)
{
    Q_UNUSED(initialDataValues)

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }
    m_node = rclcpp::Node::make_shared(m_name + "DataModel");
    m_clientDistance = m_node->create_client<navigation_interfaces::srv::CheckNearToPoi>("NavigationComponent/CheckNearToPoi");
    RCLCPP_DEBUG(m_node->get_logger(), "GoToChargingStationDataModel::start");
    std::cout << "GoToChargingStationDataModel::start";
    m_thread = std::make_shared<std::thread>([this]{ start();});
    return true;
}


bool GoToChargingStationDataModel::close()
{
    rclcpp::shutdown();  
    return true;
}

void GoToChargingStationDataModel::spin()
{
    rclcpp::spin(m_node);  
}


bool GoToChargingStationDataModel::start() {
    spin();
    close();
    return true;
}

bool GoToChargingStationDataModel::go_to_poi_by_name(std::string name) {
    
    auto request = std::make_shared<navigation_interfaces::srv::GoToPoiByName::Request>();
    request->poi_name = poi_name;
    while (!m_clientDistance->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service NavigationComponent/CheckNearToPoi. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service NavigationComponent/CheckNearToPoi not available, waiting again...");
    }
    auto result = m_clientDistance->async_send_request(request);
    std::this_thread::sleep_for (std::chrono::milliseconds(100));
    if (rclcpp::spin_until_future_complete(m_node, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        return result.get()->is_ok;
    }
    return false;
}