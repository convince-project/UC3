/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

# pragma once

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <bt_interfaces/msg/action_response.hpp>
#include <bt_interfaces/srv/tick_action.hpp>
#include <bt_interfaces/srv/halt_action.hpp>
#include <mutex>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/BufferedPort.h>

class DummyAction :
    public yarp::os::PortReader,
    public rclcpp::Node
{
public:
    DummyAction(std::string name );
    ~DummyAction();

    void tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
               std::shared_ptr<bt_interfaces::srv::TickAction::Response> response);
    void halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::HaltAction::Request> request,
               [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::HaltAction::Response> response);
    bool read(yarp::os::ConnectionReader& connection) override;

private:
    uint8_t          m_status;
    std::string      m_name;
    std::mutex       m_mutex;
    yarp::os::Port   m_changeStatusPort;
    rclcpp::Service<bt_interfaces::srv::TickAction>::SharedPtr m_tickService;
    rclcpp::Service<bt_interfaces::srv::HaltAction>::SharedPtr m_haltService;

};
