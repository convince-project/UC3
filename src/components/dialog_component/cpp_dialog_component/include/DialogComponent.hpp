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
#include <dialog_interfaces/srv/manage_context.hpp>
#include <dialog_interfaces/srv/set_poi.hpp>
#include <dialog_interfaces/srv/get_state.hpp>
#include <dialog_interfaces/srv/skip_explanation.hpp>
//#include <dialog_interfaces/srv/is_speaking.hpp>
#include <yarp/sig/AudioPlayerStatus.h>
#include <yarp/dev/ISpeechTranscription.h>
#include <text_to_speech_interfaces/srv/is_speaking.hpp>
#include <text_to_speech_interfaces/srv/set_microphone.hpp>
#include <text_to_speech_interfaces/srv/speak.hpp>
#include <text_to_speech_interfaces/srv/set_language.hpp>
#include <text_to_speech_interfaces/srv/set_voice.hpp>
#include <scheduler_interfaces/srv/get_current_poi.hpp>
#include <scheduler_interfaces/srv/end_tour.hpp>
#include <scheduler_interfaces/srv/update_poi.hpp>
#include <scheduler_interfaces/srv/set_language.hpp>
#include "nlohmann/json.hpp"
#include <random>
#include <map>
#include <unordered_map>

#include "SpeechTranscriberCallback.hpp"
#include "SpeakerStatusCallback.hpp"
#include "TourStorage.h"


#include <dialog_interfaces/srv/remember_interactions.hpp> // auto-generated from the .srv file
#include <dialog_interfaces/srv/wait_for_interaction.hpp>
#include <dialog_interfaces/srv/shorten_and_speak.hpp>
#include <dialog_interfaces/srv/answer_and_speak.hpp>
#include <dialog_interfaces/srv/interpret.hpp>

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
    // void SetLanguage(const std::shared_ptr<dialog_interfaces::srv::SetLanguage::Request> request,
                        // std::shared_ptr<dialog_interfaces::srv::SetLanguage::Response> response);
    void ManageContext(const std::shared_ptr<dialog_interfaces::srv::ManageContext::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::ManageContext::Response> response);
    void SetPoi(const std::shared_ptr<dialog_interfaces::srv::SetPoi::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::SetPoi::Response> response);
    void GetState(const std::shared_ptr<dialog_interfaces::srv::GetState::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::GetState::Response> response);
    void SkipExplanation(const std::shared_ptr<dialog_interfaces::srv::SkipExplanation::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::SkipExplanation::Response> response);
    //void IsSpeaking(const std::shared_ptr<dialog_interfaces::srv::IsSpeaking::Request> request,
    //                    std::shared_ptr<dialog_interfaces::srv::IsSpeaking::Response> response);

    void WaitForInteraction(const std::shared_ptr<dialog_interfaces::srv::WaitForInteraction::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::WaitForInteraction::Response> response);

    void ShortenAndSpeak(const std::shared_ptr<dialog_interfaces::srv::ShortenAndSpeak::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::ShortenAndSpeak::Response> response);

    void AnswerAndSpeak(const std::shared_ptr<dialog_interfaces::srv::AnswerAndSpeak::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::AnswerAndSpeak::Response> response);

    void Interpret(const std::shared_ptr<dialog_interfaces::srv::Interpret::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::Interpret::Response> response);

protected:
    void SpeakFromText(std::string & text);

private:

    /*Network Wrappers*/
    // Speech Synth
    //yarp::dev::PolyDriver m_speechSynthPoly;
    //yarp::dev::ISpeechSynthesizer *m_iSpeechSynth{nullptr};
    // ChatGPT
    yarp::dev::PolyDriver m_poiChatPoly;
    yarp::dev::PolyDriver m_genericChatPoly;
    yarp::dev::PolyDriver m_museumChatPoly;
    yarp::dev::PolyDriver m_speechTranscriptionPoly;
    yarp::dev::ILLM *m_iPoiChat{nullptr};
    yarp::dev::ILLM *m_iMuseumChat{nullptr};
    yarp::dev::ILLM *m_iGenericChat{nullptr};
    yarp::dev::ISpeechTranscription  *m_iSpeechTranscription{nullptr};
    std::map<std::string,std::string> m_voicesMap;
    std::string m_poiPrompt;
    std::string m_startPrompt;
    // Microphone
    yarp::dev::PolyDriver m_audioRecorderPoly;
    yarp::dev::IAudioGrabberSound *m_iAudioGrabberSound{nullptr};

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
    rclcpp::Service<dialog_interfaces::srv::ManageContext>::SharedPtr m_manageContextService;
    rclcpp::Service<dialog_interfaces::srv::SetPoi>::SharedPtr m_setPoiService;
    rclcpp::Service<dialog_interfaces::srv::GetState>::SharedPtr m_GetStateService;
    rclcpp::Service<dialog_interfaces::srv::SkipExplanation>::SharedPtr m_SkipExplanationService;
    rclcpp::Service<dialog_interfaces::srv::WaitForInteraction>::SharedPtr m_WaitForInteractionService;
    rclcpp::Service<dialog_interfaces::srv::ShortenAndSpeak>::SharedPtr m_ShortenAndSpeakService;
    rclcpp::Service<dialog_interfaces::srv::AnswerAndSpeak>::SharedPtr m_AnswerAndSpeakService;
    // rclcpp::Service<dialog_interfaces::srv::Interpret>::SharedPtr m_InterpretService;
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
    // std::thread m_dialogThread;
    bool CommandManager(const std::string &command, std::shared_ptr<dialog_interfaces::srv::ManageContext::Response> &response);
    bool InterpretCommand(const std::string &command, PoI currentPoI, PoI genericPoI);
    void WaitForSpeakEnd();
    void EnableMicrophone();
    void DisableMicrophone();
    bool SetLanguage(const std::string &newLang);
    bool UpdatePoILLMPrompt();

    /* Internal State Machine*/
    enum State {
        IDLE = 0,
        RUNNING = 1,
        SUCCESS = 2,
        FAILURE = 3
    };

    State m_state;
    std::atomic<bool> m_skipSpeaking{false};

    // has the current interaction already been asked?
    // int m_duplicateIndex;

    // Vector of questions and replies
    // std::vector<std::tuple<std::string, std::string>>  m_questions_and_replies;

    // last received interaction TODO: make uniform with m_lastQuestion, which I think is the same
    std::string m_last_received_interaction;
    // map of vectors of replies
    std::unordered_map<std::string, std::vector<std::string>> m_replies;

    // ROS2 Client Node that leverage the RememberInteractions service to get information about the interaction
    rclcpp::Client<dialog_interfaces::srv::RememberInteractions>::SharedPtr m_interactionClient;

};

#endif
