/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#pragma once

#include <QScxmlCppDataModel>
#include <QVariantMap>
#include <rclcpp/rclcpp.hpp>
#include <QTime>
#include <QTimer>
#include <QDebug>

#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp" 
#include <optional>

using namespace std::chrono_literals;

class StartServicesDataModel: public QScxmlCppDataModel
{
    Q_SCXML_DATAMODEL

public:
    StartServicesDataModel() = default;
    bool start();
    void set_name(std::string name);
    bool setup(const QVariantMap& initialDataValues) override;
private: 
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr m_client_get_state;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr m_client_change_state;
    std::string m_name = "StartServicesDataModel";
    std::chrono::seconds m_timeout = 3s;
    std::optional<lifecycle_msgs::msg::State> m_currentState;
    bool is_client_available(const std::chrono::seconds& timeout = 3s);
    std::optional<lifecycle_msgs::msg::State> get_state(const std::chrono::seconds& timeout = 3s);
};

Q_DECLARE_METATYPE(::StartServicesDataModel*)
