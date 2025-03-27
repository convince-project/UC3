/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "StopServiceSkillDataModel.h"
#include <QDebug>

//#include <.hpp>

void StopServiceSkillDataModel::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

void StopServiceSkillDataModel::log(std::string to_log)
{
	qInfo(to_log.c_str());
}

bool StopServiceSkillDataModel::setup(const QVariantMap& initialDataValues)
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
	}

	m_node = rclcpp::Node::make_shared("StopServiceSkillDataModelNode");
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);
	//m_subscription = m_node->create_subscription<>(
	//	"/", 10, std::bind(&::topic_callback, this, std::placeholders::_1));

	return true;
}

//void StopServiceSkill::topic_callback(const ::SharedPtr msg) {
//}

