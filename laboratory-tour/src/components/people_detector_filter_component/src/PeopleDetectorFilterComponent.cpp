/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "PeopleDetectorFilterComponent.h"


bool PeopleDetectorFilterComponent::start(int argc, char*argv[])
{

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("PeopleDetectorFilterComponentNode");

    RCLCPP_DEBUG(m_node->get_logger(), "PeopleDetectorFilterComponent::start");
    m_publisher = m_node->create_publisher<std_msgs::msg::Bool>("/PeopleDetectorFilterComponent/is_followed", 10);
    m_timer = m_node->create_wall_timer(std::chrono::seconds(1), std::bind(&PeopleDetectorFilterComponent::publisher, this));
    std::cout << "PeopleDetectorFilterComponent::start";        
    return true;

}


bool PeopleDetectorFilterComponent::close()
{
    rclcpp::shutdown();
    return true;
}

void PeopleDetectorFilterComponent::spin()
{
    rclcpp::spin(m_node);  
}


void PeopleDetectorFilterComponent::publisher()
{
    std_msgs::msg::Bool msg;
    // Generate a random boolean based on the current time
    // auto now = std::chrono::steady_clock::now();
    // auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    m_counter = m_counter + 1;
    msg.data = (m_counter % 2) == 0; 
    m_publisher->publish(msg);
}

