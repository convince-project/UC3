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
#include <other_interfaces/srv/rpc_without_parameters.hpp>
 

class BatteryDrainerDataModel: public QScxmlCppDataModel
{
    Q_SCXML_DATAMODEL

public:
    BatteryDrainerDataModel() = default;
    bool drain();
    void set_name(std::string name);
    bool setup(const QVariantMap& initialDataValues) override;
private: 
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Client<other_interfaces::srv::RpcWithoutParameters>::SharedPtr m_client;
    std::string m_name = "";
};

Q_DECLARE_METATYPE(::BatteryDrainerDataModel*)
