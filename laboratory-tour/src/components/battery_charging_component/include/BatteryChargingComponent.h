/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <atomic>
#include <vector>


class BatteryChargingComponent
{
public:
    BatteryChargingComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    void BatteryStateSubscriptionCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void BatteryStatePublisherCallback();


private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr m_publisherBatteryState;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr m_subscriptionBatteryState;
    std::atomic<bool> m_batteryCharging{true};
    std::queue<double> m_lastBatteryVoltagesBeforeChange;
    std::queue<double> m_lastBatteryVoltagesAfterChange;
    bool m_changeDetected{false};
};
