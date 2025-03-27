/******************************************************************************
 *                                                                            *
 * Copyright (C) 2024 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "FaceExpressionsWrapper.hpp"
#include "yarp/os/Network.h"

using namespace std::placeholders;

bool FaceExpressionsWrapper::start(int argc, char* argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(argc, argv);
    }

    m_node = rclcpp::Node::make_shared("FaceExpressionsWrapperNode");

    m_setExpressionService = m_node->create_service<face_expressions_interfaces::srv::SetExpression>("/FaceExpressionsWrapper/SetExpression",
                                                                                                        std::bind(&FaceExpressionsWrapper::SetExpression,
                                                                                                        this,
                                                                                                        std::placeholders::_1,
                                                                                                        std::placeholders::_2));
    m_timer = m_node->create_wall_timer(m_period, [this]()->void{
        auto data = m_audioStatusPort.read();
        if (data != nullptr)
        {
            yarp::os::Bottle msg;
            msg.addString(FaceCommandMap.at(FaceCommand::START_MOUTH));
            if (data->current_buffer_size > 0)
            {
                msg.addInt32(0);    // Stop drawing squiggle
            }
            else
            {
                msg.addInt32(1);    // Start drawing squiggle
            }
            m_facePort_rpc.write(msg);
        }
    });

    RCLCPP_INFO(m_node->get_logger(), "Started node FaceExpressionsWrapperNode");
    return true;
}

void FaceExpressionsWrapper::SetExpression(const std::shared_ptr<face_expressions_interfaces::srv::SetExpression::Request> request,
                            std::shared_ptr<face_expressions_interfaces::srv::SetExpression::Response> response)
{
    yDebug() << "[FaceExpressionsWrapper::SetExpression] Called service";
    yarp::os::Bottle msg;
    if (request->command_id == FaceCommand::START_EARS ||
        request->command_id == FaceCommand::START_MOUTH ||
        request->command_id == FaceCommand::START_BLINKING ||
        request->command_id == FaceCommand::ENABLE_DRAW_EYES ||
        request->command_id == FaceCommand::ENABLE_DRAW_EARS ||
        request->command_id == FaceCommand::ENABLE_DRAW_MOUTH ||
        request->command_id == FaceCommand::EMOTION )
    {
        msg.addString(FaceCommandMap.at(request->command_id));
        if (request->value.size()==0)
        {
            response->is_ok = false;
            response->error_msg = "Missing value 0 or 1 of the command";
            return;
        }
        msg.addInt32(request->value[0]);
        yDebug() << "[FaceExpressionsWrapper::SetExpression] Sending command: " << FaceCommandMap.at(request->command_id) << " " << request->value[0];
    }
    else if (request->command_id == FaceCommand::COLOUR_MOUTH ||
             request->command_id == FaceCommand::COLOR_EARS)
    {
        msg.addString(FaceCommandMap.at(request->command_id));
        if (request->value.size() < 3)
        {
            response->is_ok = false;
            response->error_msg = "Missing number of parameters for RGB values";
            return;
        }
        
        msg.addInt32(request->value[0]);
        msg.addInt32(request->value[1]);
        msg.addInt32(request->value[2]);

        yDebug() << "[FaceExpressionsWrapper::SetExpression] Sending command: " << FaceCommandMap.at(request->command_id) << " " 
                << request->value[0] << " " << request->value[1] << " " << request->value[2];
    }
    else if (request->command_id == FaceCommand::RESET_TO_DEFAULT ||
             request->command_id == FaceCommand::BLACK_SCREEN)
    {
        msg.addString(FaceCommandMap.at(request->command_id));
        yDebug() << "[FaceExpressionsWrapper::SetExpression] Sending command: " << FaceCommandMap.at(request->command_id);
    }
    else
    {
        //Unexpected ID
        response->is_ok = false;
        response->error_msg = "Invalid command_id";
        return;
    }
    // Talk to rpc
    m_facePort_rpc.write(msg);

    //Wait for answer
    msg.clear();
    m_facePort_rpc.read(msg);
    yDebug() << "[FaceExpressionsWrapper::SetExpression] Got answer: " << msg.get(0).asString();
    if (msg.get(0).asString() == "ok")
    {
        response->is_ok = true;
        response->answer = msg.get(0).asString();
    }
    else
    {
        response->is_ok = false;
        response->answer = msg.get(0).asString();
        response->error_msg = "Failed rpc call";
    }
}

bool FaceExpressionsWrapper::configure(yarp::os::ResourceFinder &rf)
{
    bool okCheck = rf.check("FACE_EXPRESSION_WRAPPER");

    if (okCheck)
    {
        yarp::os::Searchable &config = rf.findGroup("FACE_EXPRESSION_WRAPPER");
        if (config.check("local_audio"))
        {
            m_localAudioPortName = config.find("local_audio").asString();
        }
        if (config.check("remote_audio"))
        {
            m_remoteAudioPortName = config.find("remote_audio").asString();
        }
        if (config.check("local_face_rpc"))
        {
            m_localFacePortName = config.find("local_face_rpc").asString();
        }
        if (config.check("remote_face_rpc"))
        {
            m_remoteFacePortName = config.find("remote_face_rpc").asString();
        }
    }

    m_audioStatusPort.open(m_localAudioPortName);
    if (!yarp::os::Network::connect(m_remoteAudioPortName, m_localAudioPortName))
    {
        yWarning() << "[FaceExpressionsWrapper::configure] Unable to automatically connect: " << m_remoteAudioPortName << " with " << m_localAudioPortName;
    }

    m_facePort_rpc.open(m_localFacePortName);
    if (!yarp::os::Network::connect(m_remoteFacePortName, m_localFacePortName))
    {
        yWarning() << "[FaceExpressionsWrapper::configure] Unable to automatically connect: " << m_remoteFacePortName << " with " << m_localFacePortName;
    }

    return true;
}

bool FaceExpressionsWrapper::close()
{
    rclcpp::shutdown();
    m_facePort_rpc.close();
    m_audioStatusPort.close();
    return true;
}

void FaceExpressionsWrapper::spin()
{
    rclcpp::spin(m_node);
}
