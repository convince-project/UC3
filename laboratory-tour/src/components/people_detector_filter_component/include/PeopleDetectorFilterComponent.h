/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>


class PeopleDetectorFilterComponent 
{
public:
    PeopleDetectorFilterComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    void publisher();

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::mutex m_mutex;
    std::mutex m_taskMutex;
    int m_counter = 0;
};
