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
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ISpeechSynthesizer.h>
#include <yarp/dev/ILLM.h>
#include <yarp/dev/IAudioGrabberSound.h>

// Dialog Component Service Interfaces
#include <dialog_interfaces/srv/manage_context.hpp>
#include <dialog_interfaces/srv/remember_interactions.hpp> // auto-generated from the .srv file
#include <dialog_interfaces/srv/shorten_reply.hpp>
#include <dialog_interfaces/srv/answer.hpp>
#include <dialog_interfaces/srv/set_language.hpp>
#include <dialog_interfaces/srv/interpret_command.hpp>
#include <dialog_interfaces/srv/is_speaking.hpp>
#include <dialog_interfaces/srv/set_microphone.hpp>

// Dialog Component Action Interfaces
#include <dialog_interfaces/action/wait_for_interaction.hpp>
#include <dialog_interfaces/action/speak.hpp>

// YARP modules
#include <yarp/sig/AudioPlayerStatus.h>
#include <yarp/dev/ISpeechTranscription.h>

// Text to Speech Interfaces
#include <text_to_speech_interfaces/srv/is_speaking.hpp>
#include <text_to_speech_interfaces/srv/set_microphone.hpp>
#include <text_to_speech_interfaces/srv/speak.hpp>
#include <text_to_speech_interfaces/srv/set_language.hpp>
#include <text_to_speech_interfaces/srv/set_voice.hpp>

// Scheduler Interfaces
#include <scheduler_interfaces/srv/get_current_poi.hpp>
#include <scheduler_interfaces/srv/end_tour.hpp>
#include <scheduler_interfaces/srv/update_poi.hpp>
#include <scheduler_interfaces/srv/set_language.hpp>

// ExecuteDance Interfaces
#include <execute_dance_interfaces/srv/execute_dance.hpp>

#include "nlohmann/json.hpp"
#include <random>
#include <map>
#include <unordered_map>
#include "TourStorage.h"
#include "VerbalOutputBatchReader.hpp"

class DialogComponent
{
public:
    using actionWaitForInteraction = dialog_interfaces::action::WaitForInteraction;
    using GoalHandleWaitForInteraction = rclcpp_action::ServerGoalHandle<actionWaitForInteraction>;
    using actionSpeak = dialog_interfaces::action::Speak;
    using GoalHandleSpeak = rclcpp_action::ServerGoalHandle<actionSpeak>;

    DialogComponent();

    bool start(int argc, char *argv[]);
    bool close();
    void spin();
    bool ConfigureYARP(yarp::os::ResourceFinder &rf);

    void ManageContext(const std::shared_ptr<dialog_interfaces::srv::ManageContext::Request> request,
                       std::shared_ptr<dialog_interfaces::srv::ManageContext::Response> response);

    void WaitForInteraction(const std::shared_ptr<GoalHandleWaitForInteraction> goal_handle);

    void ShortenReply(const std::shared_ptr<dialog_interfaces::srv::ShortenReply::Request> request,
                      std::shared_ptr<dialog_interfaces::srv::ShortenReply::Response> response);

    void Answer(const std::shared_ptr<dialog_interfaces::srv::Answer::Request> request,
                std::shared_ptr<dialog_interfaces::srv::Answer::Response> response);

    void SetLanguage(const std::shared_ptr<dialog_interfaces::srv::SetLanguage::Request> request,
                     std::shared_ptr<dialog_interfaces::srv::SetLanguage::Response> response);

    void InterpretCommand(const std::shared_ptr<dialog_interfaces::srv::InterpretCommand::Request> request,
                          std::shared_ptr<dialog_interfaces::srv::InterpretCommand::Response> response); // Interprets the command and returns the action to be performed

    void Speak(const std::shared_ptr<GoalHandleSpeak> goal_handle); // ROS2 action server to speak

    void SetMicrophone(const std::shared_ptr<dialog_interfaces::srv::SetMicrophone::Request> request,
                       std::shared_ptr<dialog_interfaces::srv::SetMicrophone::Response> response); // Opens/closes the microphone ports

    void IsSpeaking(const std::shared_ptr<dialog_interfaces::srv::IsSpeaking::Request> request,
                    std::shared_ptr<dialog_interfaces::srv::IsSpeaking::Response> response); // Returns true if the DialogComponent is currently sending data to the speaker ports

protected:
    // Protected methods to manage internal functions and to interact with other services
    // void SpeakFromText(std::string text, std::string dance); // ROS2 service client to TextToSpeechComponent to speak the text
    bool CommandManager(const std::string &command, std::shared_ptr<dialog_interfaces::srv::ManageContext::Response> &response); // Manages the command received from the PoiChat LLM and returns the response to the caller
    void WaitForSpeakEnd();                                                                                                      // ROS2 service client to TextToSpeechComponent to get if the TTS is speaking. Wait until it is not
    bool UpdatePoILLMPrompt();                                                                                                   // Updates the prompt of the PoIChat LLM based on the current PoI. Leverages the SchedulerComponent service to get the current PoI name
    void ExecuteDance(std::string danceName, float estimatedSpeechTime);                                                         // ROS2 service client to ExecuteDanceComponent to execute the dance with the given name
private:
    // ChatGPT
    // Defines the LLM that manages the context of the conversation
    yarp::dev::PolyDriver m_poiChatPoly;
    yarp::dev::ILLM *m_iPoiChat{nullptr};
    // Defines the LLM that is specialized into R1-related questions and generic questions
    yarp::dev::PolyDriver m_genericChatPoly;
    yarp::dev::ILLM *m_iGenericChat{nullptr};
    // Defines the LLM that is specialized into museum-related questions
    yarp::dev::PolyDriver m_museumChatPoly;
    yarp::dev::ILLM *m_iMuseumChat{nullptr};

    // Map of the languages supported by the SpeechSynthesizer
    std::map<std::string, std::string> m_voicesMap;
    // Stores the PoiChat LLM prompt for the start of the tour
    std::string m_startPrompt;
    // Stores the PoIChat LLM prompt for the rest of the PoIs
    std::string m_poiPrompt;

    // Callback on SpeechToText port
    std::string m_speechToTextClientName;
    std::string m_speechToTextServerName;
    yarp::os::BufferedPort<yarp::os::Bottle> m_speechToTextPort;

    rclcpp::Node::SharedPtr m_node;

    /* ROS2 Services Servers provided by this component */
    rclcpp::Service<dialog_interfaces::srv::ManageContext>::SharedPtr m_manageContextService;
    rclcpp::Service<dialog_interfaces::srv::ShortenReply>::SharedPtr m_ShortenReplyService;
    rclcpp::Service<dialog_interfaces::srv::Answer>::SharedPtr m_AnswerService;
    rclcpp::Service<dialog_interfaces::srv::SetLanguage>::SharedPtr m_SetLanguageService;
    rclcpp::Service<dialog_interfaces::srv::InterpretCommand>::SharedPtr m_InterpretCommandService;
    rclcpp::Service<dialog_interfaces::srv::SetMicrophone>::SharedPtr m_SetMicrophoneService;
    rclcpp::Service<dialog_interfaces::srv::IsSpeaking>::SharedPtr m_IsSpeakingService;

    // ROS2 Action Server for WaitForInteraction
    rclcpp_action::Server<dialog_interfaces::action::WaitForInteraction>::SharedPtr m_WaitForInteractionAction;
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const dialog_interfaces::action::WaitForInteraction::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleWaitForInteraction> goal_handle);
    void handle_accepted(
        const std::shared_ptr<GoalHandleWaitForInteraction> goal_handle);

    // ROS2 Action Server for Speak
    rclcpp_action::Server<dialog_interfaces::action::Speak>::SharedPtr m_SpeakAction;
    rclcpp_action::GoalResponse handle_speak_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const dialog_interfaces::action::Speak::Goal> goal);
    rclcpp_action::CancelResponse handle_speak_cancel(
        const std::shared_ptr<GoalHandleSpeak> goal_handle);
    void handle_speak_accepted(
        const std::shared_ptr<GoalHandleSpeak> goal_handle);

    /*Dialog JSON*/
    std::shared_ptr<TourStorage> m_tourStorage;
    std::string m_currentPoiName;
    std::string m_jsonPath;
    std::string m_tourName;

    // is the tour loaded at the start?
    bool m_tourLoadedAtStart;

    // Pseudo-Random Number Generator for the randomization of the predefined answers inside the JSON
    /*Answers Randomizer*/
    std::mt19937 m_random_gen;
    std::default_random_engine m_rand_engine;
    std::uniform_int_distribution<std::mt19937::result_type> m_uniform_distrib;
    int m_fallback_threshold;
    int m_fallback_repeat_counter;
    std::string m_last_valid_speak;

    /* Internal State Machine*/
    enum State
    {
        IDLE = 0,
        RUNNING = 1,
        SUCCESS = 2,
        FAILURE = 3
    };

    State m_state;

    // save the last received interaction, may be omitted
    std::string m_last_received_interaction;
    // map of vectors of replies
    std::unordered_map<std::string, std::vector<std::string>> m_replies;

    // keep track of the index of the predefined answers to be given in sequence
    int m_predefined_answer_index;
    int m_number_of_predefined_answers;

    // keep track of the predefined answer to store in conversation history
    std::string m_predefined_answer;

    VerbalOutputBatchReader m_verbalOutputBatchReader;

    // direct access to the microphone
    bool m_manualMicDisabled{false};
    yarp::dev::PolyDriver m_audioRecorderPoly;
    yarp::dev::IAudioGrabberSound *m_iAudioGrabberSound{nullptr};

    // Audio ports to play the verbal output
        bool m_startedSpeaking{false};
    std::mutex m_mutex;
    yarp::os::BufferedPort<yarp::sig::Sound> m_audioPort;
    yarp::os::BufferedPort<yarp::sig::AudioPlayerStatus> m_audioStatusPort;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_speakerStatusPub;
    rclcpp::TimerBase::SharedPtr m_timer;
};

#endif
