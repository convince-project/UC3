/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

# pragma once

#include <QScxmlCppDataModel>
#include <QVariant>
#include <string>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#define LOW_THRESHOLD   30
#define HIGH_THRESHOLD  80
#define VECTOR_SIZE     10

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
   std::vector<float> m_batteryLevelVector;
   bool m_returnValue = false;
   rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr m_subscription;
   std::shared_ptr<std::thread> m_threadSpin;
   std::shared_ptr<rclcpp::Node> m_node;
	
};

Q_DECLARE_METATYPE(::BatteryLevelSkillDataModel*)