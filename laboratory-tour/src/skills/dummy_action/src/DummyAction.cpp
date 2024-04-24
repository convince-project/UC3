/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "DummyAction.h"

#include <iostream>

DummyAction::DummyAction(std::string name, std::string default_status ) : Node(name + "Skill")
{
    m_name = name;
    std::string portName = "/"+m_name+"/rpc:i";
    if(!m_changeStatusPort.open(portName.c_str()))
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open rpc port");
    }
    m_changeStatusPort.setReader(*this);

    RCLCPP_DEBUG(this->get_logger(), "%s::start",m_name.c_str());
        auto message = bt_interfaces::msg::ActionResponse();
    if(default_status=="SUCCESS")
    {
        m_status = message.SKILL_SUCCESS;
    }
    else if(default_status=="RUNNING")
    {
        m_status = message.SKILL_RUNNING;
    }
    else if(default_status=="FAILURE")
    {
        m_status = message.SKILL_FAILURE;
    }

    m_tickService = this->create_service<bt_interfaces::srv::TickAction>(m_name + "Skill/tick",  std::bind(&DummyAction::tick,
                                                                                                                 this,
                                                                                                                 std::placeholders::_1,
                                                                                                                 std::placeholders::_2));
    m_haltService = this->create_service<bt_interfaces::srv::HaltAction>(m_name + "Skill/halt",  std::bind(&DummyAction::halt,
                                                                                                                 this,
                                                                                                                 std::placeholders::_1,
                                                                                                                 std::placeholders::_2));
}

void DummyAction::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::TickAction::Request> request,
                                       std::shared_ptr<bt_interfaces::srv::TickAction::Response>      response)
{
    RCLCPP_DEBUG(this->get_logger(), "%s::request_ack",m_name.c_str());
    response->status.status = m_status;
    response->is_ok = true;
}


void DummyAction::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::HaltAction::Request> request,
                                       [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::HaltAction::Response>      response)
{

    RCLCPP_DEBUG(this->get_logger(), "%s::send_stop",m_name.c_str());
    response->is_ok = true;
}

bool DummyAction::read(yarp::os::ConnectionReader& connection)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    yarp::os::Bottle in;
    yarp::os::Bottle out;
    bool ok = in.read(connection);
    if (!ok) return false;

    //parse string command
    if(in.get(0).isString())
    {
        std::string cmd = in.get(0).asString();
        auto message = bt_interfaces::msg::ActionResponse();
        if(cmd=="help")
        {
            out.addVocab32("many");
            out.addString("RPC commands for Dummy action\n");
            out.addString("\t setSuccess - Sets the current status to success\n");
            out.addString("\t setRunning - Sets the current status to running\n");
            out.addString("\t setFailure - Sets the current status to failure\n");
            out.addString("\t getStatus - Gets the current status\n");
            out.addString("");
        }
        else if(cmd=="setSuccess")
        {
            m_status = message.SKILL_SUCCESS;
            out.addString("ack");
        }
        else if(cmd=="setRunning")
        {
            m_status = message.SKILL_RUNNING;
            out.addString("ack");
        }
        else if(cmd=="setFailure")
        {
            m_status = message.SKILL_FAILURE;
            out.addString("ack");
        }
        else if(cmd=="getStatus")
        {
            switch (m_status)
            {
            case message.SKILL_RUNNING:
                out.addString("RUNNING");
                break;
            case message.SKILL_SUCCESS:
                out.addString("SUCCESS");
                break;
            case message.SKILL_FAILURE:
                out.addString("FAILURE");
                break;

            default:
                break;
            }
        }
    }
    else{
        out.addString("nack");
        return false;
    }

    yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender != nullptr)
    {
        out.write(*returnToSender);
    }


    return true;
}

DummyAction::~DummyAction()
{
    if(m_changeStatusPort.isOpen())
    {
        std::cout << "Closing ports...\n";
        RCLCPP_DEBUG(this->get_logger(), "Closing ports...");
        m_changeStatusPort.close();
    }
}
