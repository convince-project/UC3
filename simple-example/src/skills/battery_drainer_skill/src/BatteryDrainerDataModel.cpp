/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
#include "BatteryDrainerDataModel.h"

#include <QDebug>
#include <QTimer>
#include <QScxmlStateMachine>


void BatteryDrainerDataModel::set_name(std::string name)
{
    m_name=name; 
}

bool BatteryDrainerDataModel::setup(const QVariantMap &initialDataValues)
{
    Q_UNUSED(initialDataValues)

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }

    m_node = rclcpp::Node::make_shared(m_name + "DataModel");
    m_client = m_node->create_client<other_interfaces::srv::RpcWithoutParameters>(m_name + "Component/Drain");

    RCLCPP_DEBUG(m_node->get_logger(), "BatteryDrainerDataModel::start");
    std::cout << "BatteryDrainerDataModel::start";

    return true;
}



bool BatteryDrainerDataModel::drain() {
    auto request = std::make_shared<other_interfaces::srv::RpcWithoutParameters::Request>();
    auto result = m_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(m_node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_DEBUG(m_node->get_logger(), "BatteryDrainerDataModel::success");
    } else {
        RCLCPP_DEBUG(m_node->get_logger(), "BatteryDrainerDataModel::failure");
    }

    return true;
}