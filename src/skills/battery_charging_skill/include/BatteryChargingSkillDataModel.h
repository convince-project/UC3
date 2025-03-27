/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#pragma once

#include <QScxmlCppDataModel>
#include <QVariant>
#include <string>
#include <sensor_msgs/msg/battery_state.hpp>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#define POWER_SUPPLY_STATUS_CHARGING 1
// #define VOLTAGE_LIMIT 28.3 // Below this limit the battery is not charging


class BatteryChargingSkillDataModel: public QScxmlCppDataModel
{
    Q_SCXML_DATAMODEL

public:
    BatteryChargingSkillDataModel() = default;
    bool setup(const QVariantMap& initialDataValues) override;
    void log(std::string to_log);
    void topic_battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    static void spin(std::shared_ptr<rclcpp::Node> node);
private: 
    // sensor_msgs::msg::BatteryState m_batteryState;
    uint m_status;
    float m_voltage;
    float m_old_voltage;
    bool m_charging;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr m_subscription;
    std::shared_ptr<std::thread> m_threadSpin;
    std::shared_ptr<rclcpp::Node> m_node;

};

Q_DECLARE_METATYPE(::BatteryChargingSkillDataModel*)