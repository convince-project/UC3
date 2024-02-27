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
#include <navigation_interfaces/srv/check_near_to_poi.hpp>


class IsAtChargingStationDataModel: public QScxmlCppDataModel
{
    Q_SCXML_DATAMODEL

public:
    IsAtChargingStationDataModel() = default;
    void set_name(std::string name);
    bool is_near_to_poi(std::string poi_name, double distance);
    bool close();
    void spin();
    bool start() ;
    bool setup(const QVariantMap& initialDataValues) override;
private: 
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Client<navigation_interfaces::srv::CheckNearToPoi>::SharedPtr m_clientDistance;
    std::string m_name = "";
    std::shared_ptr<std::thread> m_thread;
};

Q_DECLARE_METATYPE(::IsAtChargingStationDataModel*)
