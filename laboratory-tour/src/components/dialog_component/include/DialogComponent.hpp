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
#include <std_msgs/msg/bool.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ISpeechSynthesizer.h>
#include <yarp/dev/ILLM.h>
#include <yarp/dev/IAudioGrabberSound.h>
#include <dialog_interfaces/srv/get_language.hpp>
#include <dialog_interfaces/srv/set_language.hpp>
#include <dialog_interfaces/srv/enable_dialog.hpp>
#include <dialog_interfaces/srv/set_poi.hpp>
#include <dialog_interfaces/srv/get_state.hpp>
#include <dialog_interfaces/srv/skip_explanation.hpp>
//#include <dialog_interfaces/srv/is_speaking.hpp>
#include <yarp/dev/AudioPlayerStatus.h>
#include <text_to_speech_interfaces/srv/is_speaking.hpp>
#include <text_to_speech_interfaces/srv/set_microphone.hpp>
#include <text_to_speech_interfaces/srv/speak.hpp>
#include <scheduler_interfaces/srv/get_current_poi.hpp>
#include <scheduler_interfaces/srv/end_tour.hpp>
#include "nlohmann/json.hpp"
#include <random>

#include "SpeechTranscriberCallback.hpp"
#include "SpeakerStatusCallback.hpp"
#include "TourStorage.h"

class DialogComponent
{
public:
    DialogComponent();

    bool start(int argc, char*argv[]);
    bool close();
    void spin();
    bool ConfigureYARP(yarp::os::ResourceFinder &rf);

    void GetLanguage([[maybe_unused]] const std::shared_ptr<dialog_interfaces::srv::GetLanguage::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::GetLanguage::Response> response);
    void SetLanguage(const std::shared_ptr<dialog_interfaces::srv::SetLanguage::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::SetLanguage::Response> response);
    void EnableDialog(const std::shared_ptr<dialog_interfaces::srv::EnableDialog::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::EnableDialog::Response> response);
    void SetPoi(const std::shared_ptr<dialog_interfaces::srv::SetPoi::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::SetPoi::Response> response);
    void GetState(const std::shared_ptr<dialog_interfaces::srv::GetState::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::GetState::Response> response);
    void SkipExplanation(const std::shared_ptr<dialog_interfaces::srv::SkipExplanation::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::SkipExplanation::Response> response);
    //void IsSpeaking(const std::shared_ptr<dialog_interfaces::srv::IsSpeaking::Request> request,
    //                    std::shared_ptr<dialog_interfaces::srv::IsSpeaking::Response> response);

private:
    /*Network Wrappers*/
    // Speech Synth
    //yarp::dev::PolyDriver m_speechSynthPoly;
    //yarp::dev::ISpeechSynthesizer *m_iSpeechSynth{nullptr};
    // ChatGPT
    yarp::dev::PolyDriver m_poiChatPoly;
    yarp::dev::PolyDriver m_genericChatPoly;
    yarp::dev::PolyDriver m_museumChatPoly;
    yarp::dev::ILLM *m_iPoiChat{nullptr};
    yarp::dev::ILLM *m_iMuseumChat{nullptr};
    yarp::dev::ILLM *m_iGenericChat{nullptr};
    std::string m_poiPrompt;
    std::string m_startPrompt;
    // Microphone
    //yarp::dev::PolyDriver m_audioRecorderPoly;
    //yarp::dev::IAudioGrabberSound *m_iAudioGrabberSound{nullptr};

    // Callback on SpeechTranscriber port
    SpeechTranscriberCallback m_speechTranscriberCallback;
    std::string m_speechTranscriberClientName;
    std::string m_speechTranscriberServerName;
    yarp::os::BufferedPort<yarp::os::Bottle> m_speechTranscriberPort;

    /*Speakers*/
    //yarp::os::BufferedPort<yarp::sig::Sound> m_speakersAudioPort;
    //yarp::os::BufferedPort<yarp::dev::AudioPlayerStatus> m_speakersStatusPort;
    //SpeakerStatusCallback m_speakerCallback;

    /*ROS2*/
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<dialog_interfaces::srv::GetLanguage>::SharedPtr m_getLanguageService;
    rclcpp::Service<dialog_interfaces::srv::SetLanguage>::SharedPtr m_setLanguageService;
    rclcpp::Service<dialog_interfaces::srv::EnableDialog>::SharedPtr m_enableDialogService;
    rclcpp::Service<dialog_interfaces::srv::SetPoi>::SharedPtr m_setPoiService;
    rclcpp::Service<dialog_interfaces::srv::GetState>::SharedPtr m_GetStateService;
    rclcpp::Service<dialog_interfaces::srv::SkipExplanation>::SharedPtr m_SkipExplanationService;
    //rclcpp::Service<dialog_interfaces::srv::IsSpeaking>::SharedPtr m_IsSpeakingService;

    rclcpp::Client<text_to_speech_interfaces::srv::IsSpeaking>::SharedPtr m_isSpeakingClient;
    rclcpp::Client<text_to_speech_interfaces::srv::Speak>::SharedPtr m_speakClient;
    rclcpp::Client<text_to_speech_interfaces::srv::SetMicrophone>::SharedPtr m_setMicrophoneClient;
    std::mutex m_mutex;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_speakerStatusPub;
    //rclcpp::TimerBase::SharedPtr m_timer;

    /*Dialog JSON*/
    std::shared_ptr<TourStorage> m_tourStorage;
    std::string m_currentPoiName;
    std::string m_jsonPath;
    std::string m_tourName;
    ///TODO: Horrible solution. Remove this as soon as possible
    std::string m_lastQuestion;
    // END
    bool m_tourLoadedAtStart;

    /*Answers Randomizer*/
    std::mt19937 m_random_gen;
    std::default_random_engine m_rand_engine;
    std::uniform_int_distribution<std::mt19937::result_type> m_uniform_distrib;
    int m_fallback_threshold;
    int m_fallback_repeat_counter;
    std::string m_last_valid_speak;

    /*Threading*/
    std::thread m_dialogThread;
    void DialogExecution();
    bool CommandManager(const std::string &command, PoI currentPoI, PoI genericPoI, std::string & phrase);
    bool InterpretCommand(const std::string &command, PoI currentPoI, PoI genericPoI, std::string & phrase);
    bool m_exit;
    void WaitForSpeakEnd();

    /* Internal State Machine*/
    enum State {
        IDLE = 0,
        RUNNING = 1,
        SUCCESS = 2,
        FAILURE = 3
    };

    State m_state;
    std::atomic<bool> m_skipSpeaking{false};
};

#endif
