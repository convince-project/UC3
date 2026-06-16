/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <atomic>
#include <queue>
#include <random>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>


class ChargeBatteryComponent
{
public:
    ChargeBatteryComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    void BatteryStateSubscriptionCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void BatteryStatePublisherCallback();
    void SetRandomBatteryCharge(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response);


private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr m_publisherBatteryState;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_setChargeService;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr m_subscriptionBatteryState;
    std::atomic<bool> m_batteryCharging{false};
    std::queue<double> m_lastBatteryVoltagesBeforeChange;
    std::queue<double> m_lastBatteryVoltagesAfterChange;
    bool m_changeDetected{false};
    yarp::os::Network m_yarp;
    yarp::os::Port m_rpcPort;
    std::mt19937 m_randomEngine{std::random_device{}()};
    std::uniform_int_distribution<int> m_randomDistribution{40, 80};
};
