/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "BatteryLevelSkillDataModel.h"
#include <QDebug>

//#include <.hpp>

void BatteryLevelSkillDataModel::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

void BatteryLevelSkillDataModel::log(std::string to_log)
{
	qInfo(to_log.c_str());
}

bool BatteryLevelSkillDataModel::setup(const QVariantMap& initialDataValues)
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
	}

	m_node = rclcpp::Node::make_shared("BatteryLevelSkillDataModelNode");
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);
	m_subscription = m_node->create_subscription<sensor_msgs::msg::BatteryState>(
     "/battery_status", 10, std::bind(&BatteryLevelSkillDataModel::topic_battery_callback, this, std::placeholders::_1));
	return true;
}

void BatteryLevelSkillDataModel::topic_battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    m_batteryLevelVector.push_back(msg->percentage);
	if(m_batteryLevelVector.size() > VECTOR_SIZE){
		m_batteryLevelVector.erase(m_batteryLevelVector.begin());
	}
	float l_percentageMean = accumulate(m_batteryLevelVector.begin(), m_batteryLevelVector.end(), 0.0)/m_batteryLevelVector.size();

	if (l_percentageMean < LOW_THRESHOLD)
	{
		if (m_returnValue == true)
		{
			m_returnValue = false;
		}
	}
	else if (l_percentageMean > HIGH_THRESHOLD)
	{
		if (m_returnValue == false)
		{
			m_returnValue = true;
		}
	}
	std::cout << "Battery return: " << m_returnValue << std::endl;
}