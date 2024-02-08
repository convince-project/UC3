/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
#include "BatteryChargingDataModel.h"

#include <QDebug>
#include <QTimer>
#include <QScxmlStateMachine>
#include <thread>


void BatteryChargingDataModel::set_name(std::string name)
{
    m_name=name; 
}

bool BatteryChargingDataModel::setup(const QVariantMap &initialDataValues)
{
    Q_UNUSED(initialDataValues)

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }
    m_node = rclcpp::Node::make_shared(m_name + "DataModel");

    m_subscription = m_node->create_subscription<sensor_msgs::msg::BatteryState>(
     "/battery_status", 10, std::bind(&BatteryChargingDataModel::topic_callback, this, std::placeholders::_1));

    RCLCPP_DEBUG(m_node->get_logger(), "BatteryChargingDataModel::start");
    std::cout << "BatteryChargingDataModel::start";
    m_thread = std::make_shared<std::thread>([this]{ start();});
    return true;
}


bool BatteryChargingDataModel::close()
{
    rclcpp::shutdown();  
    return true;
}

void BatteryChargingDataModel::spin()
{
    rclcpp::spin(m_node);  
}


bool BatteryChargingDataModel::start() {
    spin();
    close();
    return true;
}

void BatteryChargingDataModel::topic_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    m_status = msg->power_supply_status;
        // RCLCPP_DEBUG_STREAM(m_node->get_logger(), "BatteryLevel" << msg->percentage);
    // std::cout << "BatteryLevel" << msg->percentage << std::endl;

}