/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <navigation_interfaces/srv/check_near_to_poi.hpp>
#include <atomic>
#include <vector>
#include <queue>
#include <algorithm>


class BatteryChargingComponent
{
public:
    BatteryChargingComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    void BatteryStateSubscriptionCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void BatteryStatePublisherCallback();
    static double ComputeUpdatedBatteryPercentage(double current_percentage, double recharge_rate, bool is_near_charging_station, double max_percentage = 100.0, double minimum_percentage = 0.0);

private:
    void CheckChargingStationProximity();
    void PublishBatteryState();

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr m_publisherBatteryState;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr m_subscriptionBatteryState;
    rclcpp::Client<navigation_interfaces::srv::CheckNearToPoi>::SharedPtr m_clientCheckNearToPoi;
    std::shared_ptr<rclcpp::Node> m_navigationNode;
    std::atomic<bool> m_batteryCharging{false};
    std::queue<double> m_lastBatteryVoltagesBeforeChange;
    std::queue<double> m_lastBatteryVoltagesAfterChange;
    bool m_changeDetected{false};
    bool m_isNearChargingStation{false};
    double m_batteryPercentage{100.0};
    double m_rechargeRate{10.0};
};
