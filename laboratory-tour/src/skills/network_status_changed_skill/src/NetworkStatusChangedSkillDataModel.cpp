/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "NetworkStatusChangedSkillDataModel.h"
#include <QDebug>

//#include <.hpp>

void NetworkStatusChangedSkillDataModel::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

void NetworkStatusChangedSkillDataModel::log(std::string to_log)
{
	qInfo(to_log.c_str());
}

bool NetworkStatusChangedSkillDataModel::setup(const QVariantMap& initialDataValues)
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
	}

	m_node = rclcpp::Node::make_shared("NetworkStatusChangedSkillDataModelNode");
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);
	m_subscription = m_node->create_subscription<std_msgs::msg::Bool>(
		"/CheckNetworkComponent/NetworkChanged", 10, std::bind(&NetworkStatusChangedSkillDataModel::topic_callback, this, std::placeholders::_1));

	return true;
}

void NetworkStatusChangedSkillDataModel::topic_callback(const std_msgs::msg::Bool::SharedPtr msg) {
	m_edge_detected = msg->data;
}

