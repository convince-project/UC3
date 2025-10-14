/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "NarrateComponent.h"

YARP_LOG_COMPONENT(NARRATE_COMPONENT, "convince.narrate_component.NarrateComponent")

bool NarrateComponent::configureYARP(yarp::os::ResourceFinder &rf)
{
    bool okCheck = rf.check("NARRATECOMPONENT");
    if (okCheck)
    {
        yarp::os::Searchable &component_config = rf.findGroup("NARRATECOMPONENT");
        if (component_config.check("local-suffix"))
        {
            std::string local_suffix = component_config.find("local-suffix").asString();
            m_outSoundPort.open("/NarrateComponent" + local_suffix + "/outSound:o");
            m_playerStatusInPort.open("/NarrateComponent" + local_suffix + "/playerStatus:i");
        }
        else
        {
            m_outSoundPort.open("/NarrateComponent/outSound:o");
            m_playerStatusInPort.open("/NarrateComponent/playerStatus:i");
        }
    }
    else
    {
        m_outSoundPort.open("/NarrateComponent/outSound:o");
        m_playerStatusInPort.open("/NarrateComponent/playerStatus:i");
    }

    // Set the queue for the VerbalOutputBatchReader
    m_verbalOutputBatchReader.setSoundQueue(&m_soundQueue);
    m_outSoundPort.useCallback(m_verbalOutputBatchReader);

    // Set the ports for the SoundConsumerThread
    // if (!m_soundConsumerThread.setPorts(&m_outSoundPort, &m_playerStatusInPort))
    // {
    //     yError() << "[NarrateComponent::ConfigureYARP] Unable to set ports for SoundConsumerThread";
    //     return false;
    // }

    yInfo() << "[NarrateComponent::ConfigureYARP] Successfully configured component";
    return true;
}

bool NarrateComponent::start(int argc, char*argv[])
{

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("NarrateComponentNode");
    m_narrateService = m_node->create_service<narrate_interfaces::srv::Narrate>("/NarrateComponent/Narrate",
                                                                                std::bind(&NarrateComponent::Narrate,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_isDoneService = m_node->create_service<narrate_interfaces::srv::IsDone>("/NarrateComponent/IsDone",
                                                                                std::bind(&NarrateComponent::IsDone,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_stopService = m_node->create_service<narrate_interfaces::srv::Stop>("/NarrateComponent/Stop",
                                                                                std::bind(&NarrateComponent::Stop,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));

    RCLCPP_DEBUG(m_node->get_logger(), "NarrateComponent::start");
    return true;

}



bool NarrateComponent::close()
{
    rclcpp::shutdown();
    return true;
}

void NarrateComponent::spin()
{
    rclcpp::spin(m_node);
}

bool NarrateComponent::_formatPointAction(const std::string& actionParam, std::string& target)
{
    size_t delimiterPos = actionParam.find("::");
    if (delimiterPos == std::string::npos || delimiterPos + 2 >= actionParam.length())
    {
        RCLCPP_ERROR(m_node->get_logger(), "Invalid point action format: %s", actionParam.c_str());
        return false;
    }
    target = actionParam.substr(delimiterPos + 2);
    return true;
}

void NarrateComponent::_waitForPlayerStatus(bool discriminator)
{
    bool keepOnWaiting;
    do{
        yarp::sig::AudioPlayerStatus* player_status = m_playerStatusInPort.read();
        if (player_status != nullptr && player_status->current_buffer_size > 0)
        {
            keepOnWaiting = !discriminator;
        }
        else
        {
            keepOnWaiting = discriminator;
        }
    } while (keepOnWaiting);
}

bool NarrateComponent::_sendForBatchSynthesis(const std::vector<std::string>& texts)
{
    if (!m_clientSynthesizeTexts)
    {
        m_clientSynthesizeTexts = rclcpp_action::create_client<ActionSynthesizeTexts>(m_node, "/TextToSpeechComponent/BatchGenerationAction");
    }

    if (!m_clientSynthesizeTexts->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(m_node->get_logger(), "Action server not available after waiting");
        return false;
    }

    auto goal_msg = ActionSynthesizeTexts::Goal();
    goal_msg.texts = texts;

    RCLCPP_INFO(m_node->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<ActionSynthesizeTexts>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NarrateComponent::_goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NarrateComponent::_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&NarrateComponent::_result_callback, this, std::placeholders::_1);

    m_clientSynthesizeTexts->async_send_goal(goal_msg, send_goal_options);
    return true;
}

void NarrateComponent::IsDone([[maybe_unused]] const std::shared_ptr<narrate_interfaces::srv::IsDone::Request> request,
             std::shared_ptr<narrate_interfaces::srv::IsDone::Response>      response)
{
    // response->is_done = (!m_speakTask && !m_danceTask);
    response->is_done = !(m_speakTask);
    response->is_ok = true;
}

void NarrateComponent::_executeDance(std::string danceName, float estimatedSpeechTime)
{
    yInfo() << "[DialogComponent::ExecuteDance] Starting Execute Dance Service";
    auto executeDanceClientNode = rclcpp::Node::make_shared("ExecuteDanceComponentExecuteDanceNode");

    std::string pointTarget;
    if (_formatPointAction(danceName, pointTarget))
    {
        // TODO: implement point action
        RCLCPP_INFO(m_node->get_logger(), "Point action towards target: %s", pointTarget.c_str());
        return;
    }
    else
    {
        auto danceClient = executeDanceClientNode->create_client<execute_dance_interfaces::srv::ExecuteDance>("/ExecuteDanceComponent/ExecuteDance");
        auto dance_request = std::make_shared<execute_dance_interfaces::srv::ExecuteDance::Request>();
        dance_request->dance_name = danceName;
        dance_request->speech_time = estimatedSpeechTime;
        // Wait for service
        while (!danceClient->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'ExecuteDance'. Exiting.");
            }
        }
        auto dance_result = danceClient->async_send_request(dance_request);

        if (rclcpp::spin_until_future_complete(executeDanceClientNode, dance_result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execute Dance succeeded");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service execute_dance");
            return;
        }
    }
}

void NarrateComponent::_speakTask() {
    RCLCPP_INFO_STREAM(m_node->get_logger(), "New Speak Thread ");
    m_speakTask = true;
    do{
        _waitForPlayerStatus(false);
        yarp::sig::Sound sound;
        if (m_soundQueue.pop(sound))
        {
            yarp::sig::Sound& toSend = m_outSoundPort.prepare();
            toSend = sound;
            m_outSoundPort.write();
            _waitForPlayerStatus(true);
            _executeDance(m_danceBuffer[m_danceBuffer.size() - m_toSend], sound.getDuration());
            m_toSend--;
        }

    } while(!m_stopped && (!m_soundQueue.isEmpty() || m_toSend > 0));

    m_speakTask = false;
}

void NarrateComponent::_narrateTask(const std::shared_ptr<narrate_interfaces::srv::Narrate::Request> request) {
        // calls the SetCommand service
	    RCLCPP_INFO_STREAM(m_node->get_logger(), "New Narrate Thread ");
        auto setCommandClientNode = rclcpp::Node::make_shared("NarrateComponentSetCommandNode");
        std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::SetCommand>> setCommandClient =
        setCommandClientNode->create_client<scheduler_interfaces::srv::SetCommand>("/SchedulerComponent/SetCommand");
        auto setCommandRequest = std::make_shared<scheduler_interfaces::srv::SetCommand::Request>();
        setCommandRequest->command = request->command;
        while (!setCommandClient->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setCommandClient'. Exiting.");
            }
        }
        auto setCommandResult = setCommandClient->async_send_request(setCommandRequest);
        auto futureSetCommandResult = rclcpp::spin_until_future_complete(setCommandClientNode, setCommandResult);
        if (futureSetCommandResult != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error in SetCommand service");
            return;
        }
        auto setCommandResponse = setCommandResult.get();
        if (setCommandResponse->is_ok == false) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error in SetCommand service" << setCommandResponse->error_msg);

        }
        bool doneWithPoi = false;

        int actionCounter = 0;
        m_speakBuffer.clear();
        m_danceBuffer.clear();

        do {
            //calls the GetCurrentAction service
            auto getCurrentActionClientNode = rclcpp::Node::make_shared("NarrateComponentGetCurrentActionNode");
            std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::GetCurrentAction>> getCurrentActionClient =
            getCurrentActionClientNode->create_client<scheduler_interfaces::srv::GetCurrentAction>("/SchedulerComponent/GetCurrentAction");
            auto getCurrentActionRequest = std::make_shared<scheduler_interfaces::srv::GetCurrentAction::Request>();

            while (!getCurrentActionClient->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'getCurrentActionClient'. Exiting.");
                    m_errorOccurred = true;
                    return;
                }
            }
            auto getCurrentActionResult = getCurrentActionClient->async_send_request(getCurrentActionRequest);
            auto futureGetCurrentActionResult = rclcpp::spin_until_future_complete(getCurrentActionClientNode, getCurrentActionResult);
            if (futureGetCurrentActionResult != rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error in GetCurrentAction service");
                m_errorOccurred = true;
                return;
            }
            auto currentAction = getCurrentActionResult.get();
                if (currentAction->type == "speak")
                {
                    RCLCPP_INFO_STREAM(m_node->get_logger(), "Got speak action " );
                    if (actionCounter > 0 && m_speakBuffer.size() > m_danceBuffer.size()) {
                        RCLCPP_INFO_STREAM(m_node->get_logger(), "No dance action for the speak action. Putting \"gesture\"");
                        m_danceBuffer.push_back("gesture");
                    }
                    m_speakBuffer.push_back(currentAction->param);
                }
                else if (currentAction->type == "dance")
                {
                    if (actionCounter == 0 || m_danceBuffer.size() >= m_speakBuffer.size()) {
                        RCLCPP_INFO_STREAM(m_node->get_logger(), "Ignoring dance action, no speak action to go with it" );
                    }
                    else {
                        RCLCPP_INFO_STREAM(m_node->get_logger(), "Got dance action " );
                        m_danceBuffer.push_back(currentAction->param);
                    }
                }

            // calls the UpdateAction service
            auto updateActionClientNode = rclcpp::Node::make_shared("NarrateComponentUpdateActionNode");
            std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::UpdateAction>> updateActionClient =
            updateActionClientNode->create_client<scheduler_interfaces::srv::UpdateAction>("/SchedulerComponent/UpdateAction");
            auto updateActionRequest = std::make_shared<scheduler_interfaces::srv::UpdateAction::Request>();
            while (!updateActionClient->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'updateActionClient'. Exiting.");
                    m_errorOccurred = true;
                    return;
                }
            }
            auto updateActionResult = updateActionClient->async_send_request(updateActionRequest);
            auto futureUpdateActionResult = rclcpp::spin_until_future_complete(updateActionClientNode, updateActionResult);
            if (futureUpdateActionResult != rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error in UpdateAction service");
                m_errorOccurred = true;
                return;
            }
            auto updateActionResponse = updateActionResult.get();
            doneWithPoi = updateActionResponse->done_with_poi;
            if(m_stopped)
            {
                RCLCPP_INFO_STREAM(m_node->get_logger(), "Stop Command received");
                RCLCPP_INFO_STREAM(m_node->get_logger(), "Both Thread Joined");
                break;
            }
            actionCounter++;
        } while (!doneWithPoi);

        if (m_speakBuffer.size() == m_danceBuffer.size() + 1) {
            RCLCPP_INFO_STREAM(m_node->get_logger(), "No dance action for the last speak action. Putting \"gesture\"");
            m_danceBuffer.push_back("gesture");
        }
        if (m_speakBuffer.size() != m_danceBuffer.size()) {
            RCLCPP_ERROR_STREAM(m_node->get_logger(), "Error: speak and dance actions do not match. Aborting narration");
            return;
        }

        m_toSend = m_speakBuffer.size();
        m_soundQueue.flush();

        _sendForBatchSynthesis(m_speakBuffer);
        _speakTask();
}

void NarrateComponent::Narrate(const std::shared_ptr<narrate_interfaces::srv::Narrate::Request> request,
             std::shared_ptr<narrate_interfaces::srv::Narrate::Response>      response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "NarrateComponent::Narrate ");
    if (m_threadNarration.joinable()) {
        m_threadNarration.join();
    }
    m_stopped = false;
    m_errorOccurred = false;
    m_threadNarration = std::thread([this, request]() { _narrateTask(request); });
    response->is_ok = true;
}

void NarrateComponent::_goal_response_callback(const rclcpp_action::ClientGoalHandle<ActionSynthesizeTexts>::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(m_node->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(m_node->get_logger(), "Goal accepted by server, waiting for result");
        m_goalHandleSynthesizeTexts = goal_handle;
    }
}

void NarrateComponent::_feedback_callback([[maybe_unused]] rclcpp_action::ClientGoalHandle<ActionSynthesizeTexts>::SharedPtr,
                           [[maybe_unused]] const std::shared_ptr<const ActionSynthesizeTexts::Feedback> feedback)
{
    // RCLCPP_INFO(m_node->get_logger(), "Next text to be synthesized: %s", feedback->partial_text.c_str());
}

void NarrateComponent::_result_callback(const rclcpp_action::ClientGoalHandle<ActionSynthesizeTexts>::WrappedResult & result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(m_node->get_logger(), "Goal succeeded");
    }
    else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(m_node->get_logger(), "Goal was halted (canceled)");
    } else {
        RCLCPP_ERROR(m_node->get_logger(), "Goal failed");
    }
}

void NarrateComponent::Stop([[maybe_unused]] const std::shared_ptr<narrate_interfaces::srv::Stop::Request> request,
             std::shared_ptr<narrate_interfaces::srv::Stop::Response>      response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "NarrateComponent::Stop ");
    m_stopped = true;
    if (m_goalHandleSynthesizeTexts) {
        auto future_cancel = m_clientSynthesizeTexts->async_cancel_goal(m_goalHandleSynthesizeTexts);
        if (rclcpp::spin_until_future_complete(m_node, future_cancel) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to cancel goal");
        } else {
            RCLCPP_INFO(m_node->get_logger(), "Goal successfully canceled");
        }
    }
    if (m_threadNarration.joinable()) {
        m_threadNarration.join();
    }
    response->is_ok = true;
}
