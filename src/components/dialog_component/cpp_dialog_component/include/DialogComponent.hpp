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

// Dialog Component Interfaces
#include <dialog_interfaces/srv/manage_context.hpp>
#include <dialog_interfaces/srv/remember_interactions.hpp> // auto-generated from the .srv file
#include <dialog_interfaces/srv/wait_for_interaction.hpp>
#include <dialog_interfaces/srv/shorten_and_speak.hpp>
#include <dialog_interfaces/srv/answer_and_speak.hpp>
#include <dialog_interfaces/srv/set_language.hpp>
#include <dialog_interfaces/srv/interpret_command.hpp>

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


#include "nlohmann/json.hpp"
#include <random>
#include <map>
#include <unordered_map>
#include "TourStorage.h"


class DialogComponent
{
public:
    DialogComponent();

    bool start(int argc, char*argv[]);
    bool close();
    void spin();
    bool ConfigureYARP(yarp::os::ResourceFinder &rf);
    
    void ManageContext(const std::shared_ptr<dialog_interfaces::srv::ManageContext::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::ManageContext::Response> response);

    void WaitForInteraction(const std::shared_ptr<dialog_interfaces::srv::WaitForInteraction::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::WaitForInteraction::Response> response);

    void ShortenAndSpeak(const std::shared_ptr<dialog_interfaces::srv::ShortenAndSpeak::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::ShortenAndSpeak::Response> response);

    void AnswerAndSpeak(const std::shared_ptr<dialog_interfaces::srv::AnswerAndSpeak::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::AnswerAndSpeak::Response> response);

    void SetLanguage(const std::shared_ptr<dialog_interfaces::srv::SetLanguage::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::SetLanguage::Response> response);
    
    void InterpretCommand(const std::shared_ptr<dialog_interfaces::srv::InterpretCommand::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::InterpretCommand::Response> response); // Interprets the command and returns the action to be performed

protected:
    // Protected methods to manage internal functions and to interact with other services
    void SpeakFromText(std::string text); // ROS2 service client to TextToSpeechComponent to speak the text
    bool CommandManager(const std::string &command, std::shared_ptr<dialog_interfaces::srv::ManageContext::Response> &response); // Manages the command received from the PoiChat LLM and returns the response to the caller
    void WaitForSpeakEnd(); // ROS2 service client to TextToSpeechComponent to get if the TTS is speaking. Wait until it is not
    bool UpdatePoILLMPrompt(); // Updates the prompt of the PoIChat LLM based on the current PoI. Leverages the SchedulerComponent service to get the current PoI name

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
    std::map<std::string,std::string> m_voicesMap;
    // Stores the PoiChat LLM prompt for the start of the tour
    std::string m_startPrompt;
    // Stores the PoIChat LLM prompt for the rest of the PoIs
    std::string m_poiPrompt;

    // Callback on SpeechToText port
    std::string m_speechToTextClientName;
    std::string m_speechToTextServerName;
    yarp::os::BufferedPort<yarp::os::Bottle> m_speechToTextPort;

    /* ROS2 Services Servers provided by this component */
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<dialog_interfaces::srv::WaitForInteraction>::SharedPtr m_WaitForInteractionService;
    rclcpp::Service<dialog_interfaces::srv::ManageContext>::SharedPtr m_manageContextService;
    rclcpp::Service<dialog_interfaces::srv::ShortenAndSpeak>::SharedPtr m_ShortenAndSpeakService;
    rclcpp::Service<dialog_interfaces::srv::AnswerAndSpeak>::SharedPtr m_AnswerAndSpeakService;
    rclcpp::Service<dialog_interfaces::srv::SetLanguage>::SharedPtr m_SetLanguageService;
    rclcpp::Service<dialog_interfaces::srv::InterpretCommand>::SharedPtr m_InterpretCommandService;


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
    enum State {
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

};

#endif
