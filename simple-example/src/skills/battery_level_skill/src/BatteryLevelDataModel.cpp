/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
#include "BatteryLevelDataModel.h"

#include <QDebug>
#include <QTimer>
#include <QScxmlStateMachine>
#include <thread>


void BatteryLevelDataModel::set_name(std::string name)
{
    m_name=name; 
}

bool BatteryLevelDataModel::setup(const QVariantMap &initialDataValues)
{
    Q_UNUSED(initialDataValues)

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }
    m_node = rclcpp::Node::make_shared(m_name + "DataModel");

    m_subscription = m_node->create_subscription<sensor_msgs::msg::BatteryState>(
     "/battery_status", 10, std::bind(&BatteryLevelDataModel::topic_callback, this, std::placeholders::_1));

    RCLCPP_DEBUG(m_node->get_logger(), "BatteryLevelDataModel::start");
    std::cout << "BatteryLevelDataModel::start";
    m_thread = std::make_shared<std::thread>([this]{ start();});
    return true;
}


bool BatteryLevelDataModel::close()
{
    rclcpp::shutdown();  
    return true;
}

void BatteryLevelDataModel::spin()
{
    rclcpp::spin(m_node);  
}


bool BatteryLevelDataModel::start() {
    spin();
    close();
    return true;
}

void BatteryLevelDataModel::topic_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    m_level = msg->percentage;
        // RCLCPP_DEBUG_STREAM(m_node->get_logger(), "BatteryLevel" << msg->percentage);
    // std::cout << "BatteryLevel" << msg->percentage << std::endl;

}