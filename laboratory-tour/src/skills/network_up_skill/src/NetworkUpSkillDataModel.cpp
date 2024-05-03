/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "NetworkUpSkillDataModel.h"
#include <QDebug>

//#include <.hpp>

void NetworkUpSkillDataModel::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

void NetworkUpSkillDataModel::log(std::string to_log)
{
	qInfo(to_log.c_str());
}

bool NetworkUpSkillDataModel::setup(const QVariantMap& initialDataValues)
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
	}

	m_node = rclcpp::Node::make_shared("NetworkUpSkillDataModelNode");
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);
	m_subscription = m_node->create_subscription<std_msgs::msg::Bool>(
		"/CheckNetworkComponent/NetworkStatus", 10, std::bind(&NetworkUpSkillDataModel::topic_callback, this, std::placeholders::_1));

	return true;
}

void NetworkUpSkillDataModel::topic_callback(const std_msgs::msg::Bool::SharedPtr msg) {
	m_status = msg->data;
}

