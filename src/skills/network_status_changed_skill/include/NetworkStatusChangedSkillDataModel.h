/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

# pragma once

#include <QScxmlCppDataModel>
#include <QVariant>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#define EDGE_DETECTED true

class NetworkStatusChangedSkillDataModel: public QScxmlCppDataModel
{
    Q_SCXML_DATAMODEL

public:
   NetworkStatusChangedSkillDataModel() = default;
   bool setup(const QVariantMap& initialDataValues) override;
   void log(std::string to_log);
   void topic_callback(const std_msgs::msg::Bool::SharedPtr msg);
   static void spin(std::shared_ptr<rclcpp::Node> node);

private:
   bool m_status{false};
   bool m_edge_detected;
   bool m_old_status{false};
   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription;
   std::shared_ptr<std::thread> m_threadSpin;
   std::shared_ptr<rclcpp::Node> m_node;
	
};

Q_DECLARE_METATYPE(::NetworkStatusChangedSkillDataModel*)