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
#include <std_msgs/msg/bool.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ISpeechSynthesizer.h>
#include <yarp/dev/AudioPlayerStatus.h>
#include <yarp/dev/IAudioGrabberSound.h>
#include <text_to_speech_interfaces/srv/get_language.hpp>
#include <text_to_speech_interfaces/srv/set_language.hpp>
#include <text_to_speech_interfaces/srv/speak.hpp>
#include <text_to_speech_interfaces/srv/is_speaking.hpp>
#include <text_to_speech_interfaces/srv/set_microphone.hpp>

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
    void IsSpeaking(const std::shared_ptr<text_to_speech_interfaces::srv::IsSpeaking::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::IsSpeaking::Response> response);
    void SetMicrophone(const std::shared_ptr<text_to_speech_interfaces::srv::SetMicrophone::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::SetMicrophone::Response> response);

private:
    bool m_manualMicDisabled = false;
    yarp::dev::PolyDriver m_speechSynthPoly;
    yarp::dev::ISpeechSynthesizer *m_iSpeechSynth{nullptr};
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<text_to_speech_interfaces::srv::GetLanguage>::SharedPtr m_getLanguageService;
    rclcpp::Service<text_to_speech_interfaces::srv::SetLanguage>::SharedPtr m_setLanguageService;
    rclcpp::Service<text_to_speech_interfaces::srv::Speak>::SharedPtr m_speakService;
    rclcpp::Service<text_to_speech_interfaces::srv::IsSpeaking>::SharedPtr m_IsSpeakingService;
    rclcpp::Service<text_to_speech_interfaces::srv::SetMicrophone>::SharedPtr m_SetMicrophoneService;
    std::mutex m_mutex;
    yarp::os::BufferedPort<yarp::sig::Sound> m_audioPort;
    yarp::os::BufferedPort<yarp::dev::AudioPlayerStatus> m_audioStatusPort;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_speakerStatusPub;
    rclcpp::TimerBase::SharedPtr m_timer;

    yarp::dev::PolyDriver m_audioRecorderPoly;
    yarp::dev::IAudioGrabberSound *m_iAudioGrabberSound{nullptr};
};

#endif
