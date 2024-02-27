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
#include <navigation_interfaces/srv/go_to_poi_by_name.hpp>


class GoToChargingStationDataModel: public QScxmlCppDataModel
{
    Q_SCXML_DATAMODEL

public:
    GoToChargingStationDataModel() = default;
    void set_name(std::string name);
    bool go_to_poi_by_name(std::string poi_name);
    bool close();
    void spin();
    bool start() ;
    bool setup(const QVariantMap& initialDataValues) override;
private: 
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Client<navigation_interfaces::srv::GoToPoiByName>::SharedPtr m_clientGoTo;
    std::string m_name = "";
    std::shared_ptr<std::thread> m_thread;
};

Q_DECLARE_METATYPE(::GoToChargingStationDataModel*)
