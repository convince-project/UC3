/******************************************************************************
 *                                                                            *
 * Copyright (C) 2024 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#ifndef FACE_EXPRESSIONS_WRAPPER__HPP
#define FACE_EXPRESSIONS_WRAPPER__HPP

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/AudioRecorderStatus.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>

#include <rclcpp/rclcpp.hpp>
#include <face_expressions_interfaces/srv/set_expression.hpp>

#include <mutex>
#include <string>

using namespace std::chrono_literals;

class FaceExpressionsWrapper
{
private:
    std::string m_localFacePortName = "/FaceExpressionsWrapper/rpc";
    std::string m_remoteFacePortName = "/FaceExpression/rpc";
    yarp::os::Port m_facePort_rpc;

    enum FaceCommand{
        START_EARS,             //0
        START_MOUTH,            //1
        START_BLINKING,         //2
        ENABLE_DRAW_EYES,       //3
        ENABLE_DRAW_EARS,       //4
        ENABLE_DRAW_MOUTH,      //5
        COLOUR_MOUTH,           //6
        COLOR_EARS,             //7
        RESET_TO_DEFAULT,       //8
        EMOTION,                //9
        BLACK_SCREEN            //10
    };

    std::map<int, std::string> FaceCommandMap = {
        {START_EARS, "start_ears"},
        {START_MOUTH, "start_mouth"},
        {START_BLINKING, "start_blinking"},
        {ENABLE_DRAW_EYES, "enable_draw_eyes"},
        {ENABLE_DRAW_EARS, "enable_draw_ears"},
        {ENABLE_DRAW_MOUTH, "enable_draw_mouth"},
        {COLOUR_MOUTH, "colour_mouth"},
        {COLOR_EARS, "color_ears"},
        {RESET_TO_DEFAULT, "reset_to_default"},
        {EMOTION, "emotion"},
        {BLACK_SCREEN, "black_screen"}
    };

    std::string m_localAudioPortName = "/FaceExpressionsWrapper/audioStatus:i";
    std::string m_remoteAudioPortName = "/audioPlayerWrapper/status:o";
    yarp::os::BufferedPort<yarp::dev::AudioRecorderStatus> m_audioStatusPort;

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<face_expressions_interfaces::srv::SetExpression>::SharedPtr m_setExpressionService;
    std::chrono::microseconds m_period = 100ms;
    rclcpp::TimerBase::SharedPtr m_timer;

public:
    FaceExpressionsWrapper() = default;
    ~FaceExpressionsWrapper() = default;

    void SetExpression(const std::shared_ptr<face_expressions_interfaces::srv::SetExpression::Request> request,
                            std::shared_ptr<face_expressions_interfaces::srv::SetExpression::Response> response);

    void spin();
    bool start(int argc, char* argv[]);
    bool close();
    bool configure(yarp::os::ResourceFinder &rf);
};



#endif