/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
# pragma once

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
#include <execute_dance_interfaces/srv/reset_dance.hpp>
#include <execute_dance_interfaces/srv/is_dancing.hpp>
#include <cartesian_pointing_interfaces/srv/point_at.hpp>
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
    rclcpp_action::ClientGoalHandle<ActionSynthesizeTexts>::SharedPtr m_goalHandleSynthesizeTexts;

    /**
     * @brief Waits for the player status to change based on the discriminator
     *
     * @param discriminator if false, waits until the player is not speaking; if true, waits until the player is speaking
     */
    void _waitForPlayerStatus(bool discriminator);

    /**
     * @brief Sends a batch synthesis request to the text to speech action server
     *
     * @param texts vector of strings to be synthesized
     * @return true if the request was sent successfully
     * @return false if the request failed
     */
    bool _sendForBatchSynthesis(const std::vector<std::string>& texts);

    /**
     * @brief Tries to extract a point action target from the given string
     *
     * @param actionParam action parameter to parse (e.g. "point::target")
     * @param target resulting target
     * @return true if the parsing was successful
     * @return false if the parsing failed
     */
    bool _formatPointAction(const std::string& actionParam, std::string& target);

    void _goal_response_callback(const GoalHandleSynthesizeTexts::SharedPtr & goal_handle);
    void _feedback_callback([[maybe_unused]] GoalHandleSynthesizeTexts::SharedPtr,
                            [[maybe_unused]] const std::shared_ptr<const ActionSynthesizeTexts::Feedback> feedback);
    void _result_callback(const GoalHandleSynthesizeTexts::WrappedResult & result);

    void _speakTask();
    void _narrateTask(const std::shared_ptr<narrate_interfaces::srv::Narrate::Request> request);
    void _executePointing(std::string pointingTarget);
    void _executeDance(std::string danceName, float estimatedSpeechTime);
    void _resetDance();
    // void NarrateTask(const std::shared_ptr<narrate_interfaces::srv::Narrate::Request> request);
    // rclcpp::Client<text_to_speech_interfaces::srv::Speak>::SharedPtr m_speakClient;
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
    bool m_errorOccurred{false};

    std::thread m_threadNarration;
    SoundSafeQueue m_soundQueue;
    // SoundConsumerThread m_soundConsumerThread;
    VerbalOutputBatchReader m_verbalOutputBatchReader;
    yarp::os::BufferedPort<yarp::sig::Sound> m_outSoundPort; // Port to send the sound to the player
    yarp::os::BufferedPort<yarp::sig::AudioPlayerStatus> m_playerStatusInPort;
}; // class NarrateComponent
