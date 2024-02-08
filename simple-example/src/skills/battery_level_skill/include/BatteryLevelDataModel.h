/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#pragma once
#include <rclcpp/rclcpp.hpp>

#include <QScxmlCppDataModel>
#include <QVariantMap>
#include <QTime>
#include <QTimer>
#include <QDebug>
#include <sensor_msgs/msg/battery_state.hpp>


class BatteryLevelDataModel: public QScxmlCppDataModel
{
    Q_SCXML_DATAMODEL

public:
    BatteryLevelDataModel() = default;
    void set_name(std::string name);
    void topic_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    bool close();
    void spin();
    bool start() ;
    bool setup(const QVariantMap& initialDataValues) override;
private: 
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr m_subscription;
    std::string m_name = "";
    double m_level = 100.0;
    std::shared_ptr<std::thread> m_thread;
};

Q_DECLARE_METATYPE(::BatteryLevelDataModel*)
