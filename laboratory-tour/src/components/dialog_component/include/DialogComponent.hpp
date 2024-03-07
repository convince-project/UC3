/******************************************************************************
 *                                                                            *
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#ifndef DIALOG_COMPONENT__HPP
#define DIALOG_COMPONENT__HPP

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ISpeechSynthesizer.h>
#include <yarp/dev/IChatBot.h>
#include <dialog_interfaces/srv/get_language.hpp>
#include <dialog_interfaces/srv/set_language.hpp>
#include <dialog_interfaces/srv/enable_dialog.hpp>

#include "SpeechTranscriberCallback.hpp"

class DialogComponent 
{
public:
    DialogComponent();

    bool start(int argc, char*argv[]);
    bool close();
    void spin();
    bool ConfigureYARP(yarp::os::ResourceFinder &rf);

    void GetLanguage(const std::shared_ptr<dialog_interfaces::srv::GetLanguage::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::GetLanguage::Response> response);
    void SetLanguage(const std::shared_ptr<dialog_interfaces::srv::SetLanguage::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::SetLanguage::Response> response);
    void EnableDialog(const std::shared_ptr<dialog_interfaces::srv::EnableDialog::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::EnableDialog::Response> response);

private:
    yarp::dev::PolyDriver m_speechSynthPoly;
    yarp::dev::ISpeechSynthesizer *m_iSpeechSynth{nullptr};
    yarp::dev::PolyDriver m_chatBotPoly;
    yarp::dev::ISpeechSynthesizer *m_iChatBot{nullptr};
    SpeechTranscriberCallback m_speechTranscriberCallback;
    std::string m_speechTranscriberClientName;
    std::string m_speechTranscriberServerName;
    yarp::os::BufferedPort<yarp::os::Bottle> m_speechTranscriberPort;

    /*ROS2*/
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<dialog_interfaces::srv::GetLanguage>::SharedPtr m_getLanguageService;
    rclcpp::Service<dialog_interfaces::srv::SetLanguage>::SharedPtr m_setLanguageService;
    rclcpp::Service<dialog_interfaces::srv::EnableDialog>::SharedPtr m_enableDialogService;
    std::mutex m_mutex;
};

#endif