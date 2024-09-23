/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <atomic>
#include <vector>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>


class BatteryComponent
{
public:
    BatteryComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    bool getCharge(int&);
    void BatteryStatePublisherCallback();


private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_publisherBatteryState;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::atomic<bool> m_batteryCharging{false};
    std::queue<double> m_lastBatteryVoltagesBeforeChange;
    std::queue<double> m_lastBatteryVoltagesAfterChange;
    bool m_changeDetected{false};
};
