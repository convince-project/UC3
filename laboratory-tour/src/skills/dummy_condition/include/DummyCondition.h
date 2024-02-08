/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

# pragma once

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <bt_interfaces/msg/condition_response.hpp>
#include <bt_interfaces/srv/tick_condition.hpp>
#include <mutex>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/BufferedPort.h>

class DummyCondition :
    public yarp::os::PortReader,
    public rclcpp::Node
{
public:
    DummyCondition(std::string name );
    ~DummyCondition();

    void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickCondition::Request> request,
               std::shared_ptr<bt_interfaces::srv::TickCondition::Response> response);
    bool read(yarp::os::ConnectionReader& connection) override;

private:
    uint8_t          m_status;
    std::string      m_name;
    std::mutex       m_mutex;
    yarp::os::Port   m_changeStatusPort;
    rclcpp::Service<bt_interfaces::srv::TickCondition>::SharedPtr m_tickService;

};
