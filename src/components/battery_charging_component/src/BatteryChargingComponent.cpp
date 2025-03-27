/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "BatteryChargingComponent.h"

bool BatteryChargingComponent::start(int argc, char*argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("BatteryChargingComponentNode");
    m_subscriptionBatteryState = m_node->create_subscription<sensor_msgs::msg::BatteryState>("/battery_status", 
                                                                                             10, 
                                                                                             std::bind(&BatteryChargingComponent::BatteryStateSubscriptionCallback, 
                                                                                                       this, 
                                                                                                       std::placeholders::_1)
                                                                                             );
    m_publisherBatteryState = m_node->create_publisher<sensor_msgs::msg::BatteryState>("/battery_charging", 10);
    m_timer = m_node->create_wall_timer(std::chrono::seconds(1), std::bind(&BatteryChargingComponent::BatteryStatePublisherCallback, this));
    RCLCPP_INFO(m_node->get_logger(), "BatteryChargingComponent::start");
    return true;

}

bool BatteryChargingComponent::close()
{
    rclcpp::shutdown();  
    return true;
}

void BatteryChargingComponent::spin()
{
    rclcpp::spin(m_node);  
}


void BatteryChargingComponent::BatteryStateSubscriptionCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
     RCLCPP_INFO_STREAM(m_node->get_logger(), "BatteryChargingComponent::BatteryStateSubscriptionCallback" << msg->voltage);
    // if there is no data in the queue, push the first value
    if (m_lastBatteryVoltagesBeforeChange.empty()) {
        m_lastBatteryVoltagesBeforeChange.push(msg->voltage);
    } else {
        double difference = msg->voltage - m_lastBatteryVoltagesBeforeChange.back();
        if (abs(difference) > 0.45) {
            // detected a change in the battery voltage
            m_changeDetected = true;
            RCLCPP_INFO(m_node->get_logger(), "Detected a change in the battery voltage");
        }
        if (m_changeDetected) {
            if (m_lastBatteryVoltagesAfterChange.size() < 5) {
                m_lastBatteryVoltagesAfterChange.push(msg->voltage);
            } else {
                RCLCPP_INFO(m_node->get_logger(), "queues are full, checking for battery charging status");
                double averageVoltageBeforeChange = 0;
                for (int i = 0; i < 5; i++) {
                    averageVoltageBeforeChange += m_lastBatteryVoltagesBeforeChange.front();
                    m_lastBatteryVoltagesBeforeChange.pop();
                }
                averageVoltageBeforeChange /= 5;
                double averageVoltageAfterChange = 0;
                for (int i = 0; i < 5; i++) {
                    averageVoltageAfterChange += m_lastBatteryVoltagesAfterChange.front();
                    m_lastBatteryVoltagesAfterChange.pop();
                }
                averageVoltageAfterChange /= 5;
                RCLCPP_INFO_STREAM(m_node->get_logger(), "averageVoltageBeforeChange: " <<  averageVoltageBeforeChange << " averageVoltageAfterChange: " << averageVoltageAfterChange);
                double differenceAveraged = averageVoltageAfterChange - averageVoltageBeforeChange;
                if (differenceAveraged > 0.45) {
                    // detected a change in the battery voltage
                    m_batteryCharging.store(true);
                } else if (differenceAveraged < -0.45) {
                    // detected a change in the battery voltage
                    m_batteryCharging.store(false);
                }
                m_changeDetected = false;
            }
        } else {
            if (m_lastBatteryVoltagesBeforeChange.size() < 5) {
                m_lastBatteryVoltagesBeforeChange.push(msg->voltage);
            } else {
                m_lastBatteryVoltagesBeforeChange.pop();
                m_lastBatteryVoltagesBeforeChange.push(msg->voltage);
            }
        }
    }
}

void BatteryChargingComponent::BatteryStatePublisherCallback()
{
    sensor_msgs::msg::BatteryState msg;
    if (m_batteryCharging.load()) {
        msg.power_supply_status = msg.POWER_SUPPLY_STATUS_CHARGING;
    } else {
        msg.power_supply_status = msg.POWER_SUPPLY_STATUS_DISCHARGING;
    }
    m_publisherBatteryState->publish(msg);
}
