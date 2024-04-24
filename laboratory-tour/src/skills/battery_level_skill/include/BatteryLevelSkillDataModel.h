/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#pragma once

#include <QScxmlCppDataModel>
#include <QVariant>
#include <sensor_msgs/msg/battery_state.hpp>
#include <string>
#include <sensor_msgs/msg/battery_state.hpp>
#include <thread>
#include <rclcpp/rclcpp.hpp>


class BatteryLevelSkillDataModel: public QScxmlCppDataModel
{
    Q_SCXML_DATAMODEL

public:
    BatteryLevelSkillDataModel() = default;
    bool setup(const QVariantMap& initialDataValues) override;
    void log(std::string to_log);
    void topic_battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    static void spin(std::shared_ptr<rclcpp::Node> node);
private: 
    // sensor_msgs::msg::BatteryState m_batteryState;
    double percentage;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr m_subscription;
    std::shared_ptr<std::thread> m_threadSpin;
    std::shared_ptr<rclcpp::Node> m_node;

};

Q_DECLARE_METATYPE(::BatteryLevelSkillDataModel*)
