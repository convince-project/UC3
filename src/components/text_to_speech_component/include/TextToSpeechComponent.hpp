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
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ISpeechSynthesizer.h>
#include <yarp/sig/AudioPlayerStatus.h>
#include <yarp/dev/IAudioGrabberSound.h>
#include <text_to_speech_interfaces/srv/get_language.hpp>
#include <text_to_speech_interfaces/srv/set_language.hpp>
#include <text_to_speech_interfaces/srv/set_voice.hpp>
#include <text_to_speech_interfaces/srv/speak.hpp>
#include <text_to_speech_interfaces/srv/is_speaking.hpp>
#include <text_to_speech_interfaces/srv/set_microphone.hpp>
#include <text_to_speech_interfaces/action/batch_generation.hpp>

class TextToSpeechComponent
{
public:

    using actionBatchGeneration = text_to_speech_interfaces::action::BatchGeneration;
    using GoalHandleBatchGeneration = rclcpp_action::ServerGoalHandle<actionBatchGeneration>;

    TextToSpeechComponent() = default;

    bool start(int argc, char*argv[]);
    bool close();
    void spin();
    bool ConfigureYARP(yarp::os::ResourceFinder &rf);

    void GetLanguage(const std::shared_ptr<text_to_speech_interfaces::srv::GetLanguage::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::GetLanguage::Response> response);
    void SetLanguage(const std::shared_ptr<text_to_speech_interfaces::srv::SetLanguage::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::SetLanguage::Response> response);
    void SetVoice(const std::shared_ptr<text_to_speech_interfaces::srv::SetVoice::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::SetVoice::Response> response);
    void Speak(const std::shared_ptr<text_to_speech_interfaces::srv::Speak::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::Speak::Response> response);
    void IsSpeaking(const std::shared_ptr<text_to_speech_interfaces::srv::IsSpeaking::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::IsSpeaking::Response> response);
    void SetMicrophone(const std::shared_ptr<text_to_speech_interfaces::srv::SetMicrophone::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::SetMicrophone::Response> response);
    void BatchGeneration(const std::shared_ptr<GoalHandleBatchGeneration> goal_handle);

private:
    bool m_manualMicDisabled{false};
    bool m_startedSpeaking{false};
    yarp::dev::PolyDriver m_speechSynthPoly;
    yarp::dev::ISpeechSynthesizer *m_iSpeechSynth{nullptr};
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<text_to_speech_interfaces::srv::GetLanguage>::SharedPtr m_getLanguageService;
    rclcpp::Service<text_to_speech_interfaces::srv::SetLanguage>::SharedPtr m_setLanguageService;
    rclcpp::Service<text_to_speech_interfaces::srv::SetVoice>::SharedPtr m_setVoiceService;
    rclcpp::Service<text_to_speech_interfaces::srv::Speak>::SharedPtr m_speakService;
    rclcpp::Service<text_to_speech_interfaces::srv::IsSpeaking>::SharedPtr m_IsSpeakingService;
    rclcpp::Service<text_to_speech_interfaces::srv::SetMicrophone>::SharedPtr m_SetMicrophoneService;


    rclcpp_action::Server<actionBatchGeneration>::SharedPtr m_BatchGenerationAction;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const actionBatchGeneration::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleBatchGeneration> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleBatchGeneration> goal_handle);

    std::mutex m_mutex;
    yarp::os::BufferedPort<yarp::sig::Sound> m_audioPort;
    yarp::os::BufferedPort<yarp::sig::Sound> m_batchAudioPort;
    yarp::os::BufferedPort<yarp::sig::AudioPlayerStatus> m_audioStatusPort;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_speakerStatusPub;
    rclcpp::TimerBase::SharedPtr m_timer;

    yarp::dev::PolyDriver m_audioRecorderPoly;
    yarp::dev::IAudioGrabberSound *m_iAudioGrabberSound{nullptr};
};

#endif