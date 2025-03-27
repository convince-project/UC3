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

class StopServiceSkillDataModel: public QScxmlCppDataModel
{
    Q_SCXML_DATAMODEL

public:
   StopServiceSkillDataModel() = default;
   bool setup(const QVariantMap& initialDataValues) override;
   void log(std::string to_log);
   //void topic_callback(const ::SharedPtr msg);
   static void spin(std::shared_ptr<rclcpp::Node> node);

private:
   uint m_status;
   //rclcpp::Subscription<>::SharedPtr m_subscription;
   std::shared_ptr<std::thread> m_threadSpin;
   std::shared_ptr<rclcpp::Node> m_node;
	
};

Q_DECLARE_METATYPE(::StopServiceSkillSkillDataModel*)