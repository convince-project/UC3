/******************************************************************************
 *                                                                            *
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
#ifndef TEXT_TO_SPEECH_COMPONENT__HPP
#define TEXT_TO_SPEECH_COMPONENT__HPP

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ISpeechSynthesizer.h>
#include <text_to_speech_interfaces/srv/get_language.hpp>
#include <text_to_speech_interfaces/srv/set_language.hpp>
#include <text_to_speech_interfaces/srv/speak.hpp>
#include <AudioDeviceHelper.hpp>

class TextToSpeechComponent 
{
public:
    TextToSpeechComponent() = default;

    bool start(int argc, char*argv[]);
    bool close();
    void spin();
    bool ConfigureYARP(yarp::os::ResourceFinder &rf);

    void GetLanguage(const std::shared_ptr<text_to_speech_interfaces::srv::GetLanguage::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::GetLanguage::Response> response);
    void SetLanguage(const std::shared_ptr<text_to_speech_interfaces::srv::SetLanguage::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::SetLanguage::Response> response);
    void Speak(const std::shared_ptr<text_to_speech_interfaces::srv::Speak::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::Speak::Response> response);

private:
    yarp::dev::PolyDriver m_speechSynthPoly;
    yarp::dev::ISpeechSynthesizer *m_iSpeechSynth{nullptr};
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<text_to_speech_interfaces::srv::GetLanguage>::SharedPtr m_getLanguageService;
    rclcpp::Service<text_to_speech_interfaces::srv::SetLanguage>::SharedPtr m_setLanguageService;
    rclcpp::Service<text_to_speech_interfaces::srv::Speak>::SharedPtr m_speakService;
    std::mutex m_mutex;
    AudioDeviceHelper m_audioDeviceHelper;
};

#endif
