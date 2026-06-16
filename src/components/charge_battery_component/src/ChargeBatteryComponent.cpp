/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "ChargeBatteryComponent.h"
#include <std_srvs/srv/trigger.hpp>

bool ChargeBatteryComponent::start(int argc, char*argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("BatteryChargingComponentNode");

    m_node->declare_parameter<std::string>("yarp_remote_charge_rpc", "/battery_charge/rpc");
    m_node->declare_parameter<std::string>("yarp_local_rpc", "/BatteryChargingComponent/rpc");
    m_node->declare_parameter<std::string>("ros2_service_name", "/BatteryChargingComponent/set_charge");

    std::string localRpcPort = m_node->get_parameter("yarp_local_rpc").as_string();
    std::string remoteRpcPort = m_node->get_parameter("yarp_remote_charge_rpc").as_string();
    std::string serviceName = m_node->get_parameter("ros2_service_name").as_string();

    if (!yarp::os::Network::checkNetwork(2.0)) {
        RCLCPP_WARN(m_node->get_logger(), "YARP network not available, rpc calls will fail until it is up");
    }

    m_rpcPort.open(localRpcPort);
    if (!yarp::os::Network::connect(remoteRpcPort, localRpcPort)) {
        RCLCPP_WARN(m_node->get_logger(), "Unable to connect YARP rpc port %s -> %s", remoteRpcPort.c_str(), localRpcPort.c_str());
    }

    m_setChargeService = m_node->create_service<std_srvs::srv::Trigger>(serviceName,
                                                                        std::bind(&ChargeBatteryComponent::SetRandomBatteryCharge,
                                                                                  this,
                                                                                  std::placeholders::_1,
                                                                                  std::placeholders::_2));

    m_subscriptionBatteryState = m_node->create_subscription<sensor_msgs::msg::BatteryState>("/battery_status",
                                                                                             10,
                                                                                             std::bind(&ChargeBatteryComponent::BatteryStateSubscriptionCallback,
                                                                                                       this,
                                                                                                       std::placeholders::_1)
                                                                                             );
    m_publisherBatteryState = m_node->create_publisher<sensor_msgs::msg::BatteryState>("/BatteryComponent/battery_charging", 10);
    m_timer = m_node->create_wall_timer(std::chrono::seconds(1), std::bind(&ChargeBatteryComponent::BatteryStatePublisherCallback, this));
    RCLCPP_INFO(m_node->get_logger(), "ChargeBatteryComponent::start");
    return true;

}

bool ChargeBatteryComponent::close()
{
    m_rpcPort.close();
    rclcpp::shutdown();  
    return true;
}

void ChargeBatteryComponent::spin()
{
    rclcpp::spin(m_node);  
}


void ChargeBatteryComponent::BatteryStateSubscriptionCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
     RCLCPP_INFO_STREAM(m_node->get_logger(), "ChargeBatteryComponent::BatteryStateSubscriptionCallback" << msg->voltage);
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

void ChargeBatteryComponent::BatteryStatePublisherCallback()
{
    sensor_msgs::msg::BatteryState msg;
    if (m_batteryCharging.load()) {
        msg.power_supply_status = msg.POWER_SUPPLY_STATUS_CHARGING;
    } else {
        msg.power_supply_status = msg.POWER_SUPPLY_STATUS_DISCHARGING;
    }
    m_publisherBatteryState->publish(msg);
}

void ChargeBatteryComponent::SetRandomBatteryCharge(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void) request;
    int randomCharge = m_randomDistribution(m_randomEngine);
    yarp::os::Bottle msg;
    msg.addString("set_charge");
    msg.addInt32(randomCharge);

    yarp::os::Bottle reply;
    bool ok = false;
    if (m_rpcPort.isOpen()) {
        ok = m_rpcPort.write(msg, reply);
    }
    if (!ok) {
        response->success = false;
        response->message = "Failed to send YARP rpc set_charge request";
        RCLCPP_ERROR(m_node->get_logger(), "ChargeBatteryComponent::SetRandomBatteryCharge failed to forward YARP rpc");
        return;
    }

    response->success = true;
    response->message = "Battery charge request forwarded: " + std::to_string(randomCharge);
    RCLCPP_INFO(m_node->get_logger(), "ChargeBatteryComponent::SetRandomBatteryCharge forwarded random charge %d", randomCharge);
}
