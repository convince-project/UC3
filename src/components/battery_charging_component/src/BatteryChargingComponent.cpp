/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "BatteryChargingComponent.h"

#include <cmath>

namespace
{
constexpr double kDefaultRechargeRate = 10.0;
constexpr double kMaxBatteryPercentage = 100.0;
constexpr double kMinBatteryPercentage = 0.0;
constexpr double kChargingStationDistanceThreshold = 0.5;
}

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
    m_publisherBatteryState = m_node->create_publisher<sensor_msgs::msg::BatteryState>("/BatteryComponent/battery_charging", 10);
    m_timer = m_node->create_wall_timer(std::chrono::seconds(1), std::bind(&BatteryChargingComponent::BatteryStatePublisherCallback, this));

    m_navigationNode = rclcpp::Node::make_shared("BatteryChargingComponentNavigationNode");
    m_clientCheckNearToPoi = m_navigationNode->create_client<navigation_interfaces::srv::CheckNearToPoi>("/NavigationComponent/CheckNearToPoi");
    m_clientGetNavigationStatus = m_navigationNode->create_client<navigation_interfaces::srv::GetNavigationStatus>("/NavigationComponent/GetNavigationStatus");

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


double BatteryChargingComponent::ComputeUpdatedBatteryPercentage(double current_percentage, double recharge_rate, bool is_near_charging_station, double max_percentage, double minimum_percentage)
{
    if (is_near_charging_station) {
        return std::clamp(current_percentage + recharge_rate, minimum_percentage, max_percentage);
    }
    return std::clamp(current_percentage, minimum_percentage, max_percentage);
}

void BatteryChargingComponent::CheckChargingStationProximity()
{
    if (!m_clientCheckNearToPoi || !m_clientCheckNearToPoi->wait_for_service(std::chrono::milliseconds(200))) {
        return;
    }

    auto request = std::make_shared<navigation_interfaces::srv::CheckNearToPoi::Request>();
    request->poi_name = "charging_station";
    request->distance = kChargingStationDistanceThreshold;
    request->angle = 360;

    auto future = m_clientCheckNearToPoi->async_send_request(request);
    if (rclcpp::spin_until_future_complete(m_navigationNode, future, std::chrono::seconds(1)) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        m_isNearChargingStation = response->is_ok && response->is_near;
        if (m_isNearChargingStation) {
            m_batteryCharging.store(true);
        }
    }
}

void BatteryChargingComponent::UpdateBatteryInfoFromNavigation()
{
    if (!m_clientGetNavigationStatus || !m_clientGetNavigationStatus->wait_for_service(std::chrono::milliseconds(200))) {
        return;
    }

    auto request = std::make_shared<navigation_interfaces::srv::GetNavigationStatus::Request>();
    auto future = m_clientGetNavigationStatus->async_send_request(request);
    if (rclcpp::spin_until_future_complete(m_navigationNode, future, std::chrono::seconds(1)) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->is_ok) {
            m_isNearChargingStation = m_isNearChargingStation || response->current_goal == "charging_station";
        }
    }
}

void BatteryChargingComponent::PublishBatteryState()
{
    m_batteryPercentage = ComputeUpdatedBatteryPercentage(m_batteryPercentage, m_rechargeRate, m_isNearChargingStation, kMaxBatteryPercentage, kMinBatteryPercentage);

    sensor_msgs::msg::BatteryState msg;
    msg.percentage = m_batteryPercentage;
    if (m_batteryCharging.load() || m_isNearChargingStation) {
        msg.power_supply_status = msg.POWER_SUPPLY_STATUS_CHARGING;
    } else {
        msg.power_supply_status = msg.POWER_SUPPLY_STATUS_DISCHARGING;
    }
    m_publisherBatteryState->publish(msg);
}

void BatteryChargingComponent::BatteryStateSubscriptionCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
     RCLCPP_INFO_STREAM(m_node->get_logger(), "BatteryChargingComponent::BatteryStateSubscriptionCallback" << msg->voltage);
    if (msg->percentage > 0.0f) {
        m_batteryPercentage = msg->percentage;
    }

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

    CheckChargingStationProximity();
    UpdateBatteryInfoFromNavigation();
    PublishBatteryState();
}

void BatteryChargingComponent::BatteryStatePublisherCallback()
{
    CheckChargingStationProximity();
    UpdateBatteryInfoFromNavigation();
    PublishBatteryState();
}
