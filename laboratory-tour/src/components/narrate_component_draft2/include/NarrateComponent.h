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

#include <narrate_interfaces/srv/narrate.hpp>
#include <narrate_interfaces/srv/is_done.hpp>
#include <narrate_interfaces/srv/stop.hpp>
#include <scheduler_interfaces/srv/update_action.hpp>
#include <scheduler_interfaces/srv/get_current_action.hpp>
#include <scheduler_interfaces/srv/set_command.hpp>
#include <text_to_speech_interfaces/srv/speak.hpp>
#include <text_to_speech_interfaces/srv/is_speaking.hpp>
#include <execute_dance_interfaces/srv/execute_dance.hpp>
#include <execute_dance_interfaces/srv/is_dancing.hpp>

#define SERVICE_TIMEOUT 8

class NarrateComponent 
{
public:
    NarrateComponent() = default;

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
    void addTextToSpeakBuffer(const std::string& text);
    
    void speakTask();
    void danceTask();
    void NarrateTask(const std::shared_ptr<narrate_interfaces::srv::Narrate::Request> request);
    // void NarrateTask(const std::shared_ptr<narrate_interfaces::srv::Narrate::Request> request);
    // rclcpp::Client<text_to_speech_interfaces::srv::Speak>::SharedPtr m_speakClient;
    std::mutex m_speakMutex;
    std::mutex m_danceMutex;
    std::mutex m_timeMutex;
    int m_seconds_left;
    std::queue<std::string> m_speakBuffer;
    std::queue<std::string> m_danceBuffer;
    int32_t m_currentPoi;
    bool m_doneWithPoi{false};
    bool m_isSpeaking{false};
    bool m_isDancing{false};
    bool m_speakTask{false};
    bool m_danceTask{false};
    bool m_stopped = false;
    std::thread m_threadNarration;
};
