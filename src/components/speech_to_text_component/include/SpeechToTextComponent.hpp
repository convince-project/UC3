/******************************************************************************
 *                                                                            *
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
#ifndef SPEECH_TO_TEXT_COMPONENT__HPP
#define SPEECH_TO_TEXT_COMPONENT__HPP

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ISpeechTranscription.h>
#include <yarp/sig/AudioPlayerStatus.h>
#include <yarp/dev/IAudioGrabberSound.h>
#include <text_to_speech_interfaces/srv/get_language.hpp>
#include <text_to_speech_interfaces/srv/set_language.hpp>

class SpeechToTextComponent : public yarp::os::TypedReaderCallback<yarp::sig::Sound>
{
public:
    SpeechToTextComponent() = default;

    bool start(int argc, char*argv[]);
    bool close();
    void spin();
    bool ConfigureYARP(yarp::os::ResourceFinder &rf);

    void GetLanguage(const std::shared_ptr<text_to_speech_interfaces::srv::GetLanguage::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::GetLanguage::Response> response);
    void SetLanguage(const std::shared_ptr<text_to_speech_interfaces::srv::SetLanguage::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::SetLanguage::Response> response);

    void onRead(yarp::sig::Sound &msg) override;

private:
    yarp::dev::PolyDriver m_speechTranscrPoly;
    yarp::dev::ISpeechTranscription *m_iSpeechTranscr{nullptr};
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<text_to_speech_interfaces::srv::GetLanguage>::SharedPtr m_getLanguageService;
    rclcpp::Service<text_to_speech_interfaces::srv::SetLanguage>::SharedPtr m_setLanguageService;
    std::mutex m_mutex;
    yarp::os::BufferedPort<yarp::sig::Sound> m_audioInputPort;
    yarp::os::BufferedPort<yarp::os::Bottle> m_transcriptionOutputPort;

    yarp::dev::PolyDriver m_audioRecorderPoly;
    yarp::dev::IAudioGrabberSound *m_iAudioGrabberSound{nullptr};
};

#endif // SPEECH_TO_TEXT_COMPONENT__HPP
