/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
# pragma once

#include <mutex>
#include <thread>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>
#include <time.h>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Sound.h>
#include <yarp/sig/AudioPlayerStatus.h>

#include <narrate_interfaces/srv/narrate.hpp>
#include <narrate_interfaces/srv/is_done.hpp>
#include <narrate_interfaces/srv/stop.hpp>
#include <scheduler_interfaces/srv/update_action.hpp>
#include <scheduler_interfaces/srv/get_current_action.hpp>
#include <scheduler_interfaces/srv/set_command.hpp>
#include <text_to_speech_interfaces/srv/speak.hpp>
#include <text_to_speech_interfaces/srv/is_speaking.hpp>
#include <text_to_speech_interfaces/action/batch_generation.hpp>
#include <execute_dance_interfaces/srv/execute_dance.hpp>
#include <execute_dance_interfaces/srv/is_dancing.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "VerbalOutputBatchReader.hpp"

#define SERVICE_TIMEOUT 2

class NarrateComponent
{
public:
    using ActionSynthesizeTexts = text_to_speech_interfaces::action::BatchGeneration;
	using GoalHandleSynthesizeTexts = rclcpp_action::ClientGoalHandle<ActionSynthesizeTexts>;

    NarrateComponent() = default;

    bool configureYARP(yarp::os::ResourceFinder &rf);
    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    void Narrate( const std::shared_ptr<narrate_interfaces::srv::Narrate::Request> request,
                std::shared_ptr<narrate_interfaces::srv::Narrate::Response>      response);
    void IsDone([[maybe_unused]] const std::shared_ptr<narrate_interfaces::srv::IsDone::Request> request,
                std::shared_ptr<narrate_interfaces::srv::IsDone::Response>      response);
    void Stop([[maybe_unused]] const std::shared_ptr<narrate_interfaces::srv::Stop::Request> request,
                std::shared_ptr<narrate_interfaces::srv::Stop::Response>      response);

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<narrate_interfaces::srv::Narrate>::SharedPtr m_narrateService;
    rclcpp::Service<narrate_interfaces::srv::IsDone>::SharedPtr m_isDoneService;
    rclcpp::Service<narrate_interfaces::srv::Stop>::SharedPtr m_stopService;
    std::shared_ptr<rclcpp_action::Client<ActionSynthesizeTexts>> m_clientSynthesizeTexts;

    // Wait until the player is not speaking if discriminator is false
    // Wait until the player is speaking if discriminator is true
    void _waitForPlayerStatus(bool discriminator);

    bool _sendForBatchSynthesis(const std::vector<std::string>& texts);

    void goal_response_callback(const GoalHandleSynthesizeTexts::SharedPtr & goal_handle);
    void feedback_callback([[maybe_unused]] GoalHandleSynthesizeTexts::SharedPtr,
                           [[maybe_unused]] const std::shared_ptr<const ActionSynthesizeTexts::Feedback> feedback);
    void result_callback(const GoalHandleSynthesizeTexts::WrappedResult & result);

    void speakTask();
    void NarrateTask(const std::shared_ptr<narrate_interfaces::srv::Narrate::Request> request);
    void ExecuteDance(std::string danceName, float estimatedSpeechTime);
    // void NarrateTask(const std::shared_ptr<narrate_interfaces::srv::Narrate::Request> request);
    // rclcpp::Client<text_to_speech_interfaces::srv::Speak>::SharedPtr m_speakClient;
    std::mutex m_timeMutex;
    int m_seconds_left;
    size_t m_toSend;
    std::vector<std::string> m_speakBuffer;
    std::vector<std::string> m_danceBuffer;
    int32_t m_currentPoi;
    bool m_doneWithPoi{false};
    bool m_isSpeaking{false};
    bool m_speakTask{false};
    bool m_stopped{false};
    bool m_failed{false};

    std::thread m_threadNarration;
    SoundSafeQueue m_soundQueue;
    // SoundConsumerThread m_soundConsumerThread;
    VerbalOutputBatchReader m_verbalOutputBatchReader;
    yarp::os::BufferedPort<yarp::sig::Sound> m_outSoundPort; // Port to send the sound to the player
    yarp::os::BufferedPort<yarp::sig::AudioPlayerStatus> m_playerStatusInPort;
}; // class NarrateComponent
