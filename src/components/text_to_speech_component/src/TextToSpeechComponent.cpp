/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "yarp/sig/Sound.h"
#include "yarp/os/Time.h"

#include "TextToSpeechComponent.hpp"

using namespace std::chrono_literals;

bool TextToSpeechComponent::ConfigureYARP(yarp::os::ResourceFinder &rf)
{
    bool okCheck = rf.check("SPEECHSYNTHESIZER-CLIENT");
    std::string device = "speechSynthesizer_nwc_yarp";
    std::string local = "/TextToSpeechComponent/speechClient";
    std::string remote = "/speechSynthesizer_nws";

    if (okCheck)
    {
        yarp::os::Searchable &speech_config = rf.findGroup("SPEECHSYNTHESIZER-CLIENT");
        if (speech_config.check("device"))
        {
            device = speech_config.find("device").asString();
        }
        if (speech_config.check("local-suffix"))
        {
            local = "/TextToSpeechComponent" + speech_config.find("local-suffix").asString();
        }
        if (speech_config.check("remote"))
        {
            remote = speech_config.find("remote").asString();
        }
    }

    yarp::os::Property prop;
    prop.put("device", device);
    prop.put("local", local);
    prop.put("remote", remote);

    m_speechSynthPoly.open(prop);
    if (!m_speechSynthPoly.isValid())
    {
        yError() << "[TextToSpeechComponent::ConfigureYARP] Error opening speech synthesizer Client PolyDriver. Check parameters";
        return false;
    }
    m_speechSynthPoly.view(m_iSpeechSynth);
    if (!m_iSpeechSynth)
    {
        yError() << "[TextToSpeechComponent::ConfigureYARP] Error opening iSpeechSynth interface. Device not available";
        return false;
    }

    // ---------------------SPEAKERS----------------------------
    {
        std::string localAudioName = "/TextToSpeechComponent/audio:o";
        std::string localBatchAudioName = "/TextToSpeechComponent/batch:o";
        std::string statusLocalName = "/TextToSpeechComponent/audioPlayerWrapper/status:i";
        okCheck = rf.check("TEXT_TO_SPEECH_COMPONENT");
        if (okCheck)
        {
            yarp::os::Searchable &speakersConfig = rf.findGroup("TEXT_TO_SPEECH_COMPONENT");
            if (speakersConfig.check("localAudioName"))
            {
                localAudioName = speakersConfig.find("localAudioName").asString();
            }
            if (speakersConfig.check("localBatchAudioName"))
            {
                localBatchAudioName = speakersConfig.find("localBatchAudioName").asString();
            }
            if (speakersConfig.check("statusLocalName"))
            {
                statusLocalName = speakersConfig.find("statusLocalName").asString();
            }

        }

        if (!m_audioPort.open(localAudioName))
        {
            yError() << "[TextToSpeechComponent::ConfigureYARP] Unable to open port: " << localAudioName;
            return false;
        }

        if(!m_batchAudioPort.open(localBatchAudioName))
        {
            yError() << "[TextToSpeechComponent::ConfigureYARP] Unable to open port: " << localBatchAudioName;
            return false;
        }

        if (!m_audioStatusPort.open(statusLocalName))
        {
            yError() << "[TextToSpeechComponent::ConfigureYARP] Unable to open port: " << statusLocalName;
            return false;
        }
    }

    // ---------------------Microphone Activation----------------------------
    {
        okCheck = rf.check("MICROPHONE");
        device = "audioRecorder_nwc_yarp";
        local = "/DialogComponent/audio";
        remote = "/audioRecorder_nws";

        if (okCheck)
        {
            yarp::os::Searchable &mic_config = rf.findGroup("MICROPHONE");
            if (mic_config.check("device"))
            {
                device = mic_config.find("device").asString();
            }
            if (mic_config.check("local-suffix"))
            {
                local = "/DialogComponent" + mic_config.find("local-suffix").asString();
            }
            if (mic_config.check("remote"))
            {
                remote = mic_config.find("remote").asString();
            }
        }

        yarp::os::Property audioRecorder_prop;
        audioRecorder_prop.put("device", device);
        audioRecorder_prop.put("local", local);
        audioRecorder_prop.put("remote", remote);

        m_audioRecorderPoly.open(audioRecorder_prop);
        if (!m_audioRecorderPoly.isValid())
        {
            yError() << "[DialogComponent::ConfigureYARP] Error opening audioRecorder Client PolyDriver. Check parameters";
            return false;
        }
        m_audioRecorderPoly.view(m_iAudioGrabberSound);
        if (!m_iAudioGrabberSound)
        {
            yError() << "[DialogComponent::ConfigureYARP] Error opening audioRecorderSound interface. Device not available";
            return false;
        }
    }

    //Audio Device Helper
    //m_audioPort.open("/TextToSpeechComponent/audio:o");
    //m_audioStatusPort.open("/TextToSpeechComponent/audioStatus:i");
    return true;
}

rclcpp::Node::SharedPtr TextToSpeechComponent::getNode()
{
    return m_node;
}

bool TextToSpeechComponent::start(int argc, char*argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("TextToSpeechComponentNode");

    m_setLanguageService = m_node->create_service<text_to_speech_interfaces::srv::SetLanguage>("/TextToSpeechComponent/SetLanguage",
                                                                                        std::bind(&TextToSpeechComponent::SetLanguage,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));
    m_getLanguageService = m_node->create_service<text_to_speech_interfaces::srv::GetLanguage>("/TextToSpeechComponent/GetLanguage",
                                                                                        std::bind(&TextToSpeechComponent::GetLanguage,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));
    m_speakService = m_node->create_service<text_to_speech_interfaces::srv::Speak>("/TextToSpeechComponent/Speak",
                                                                                        std::bind(&TextToSpeechComponent::Speak,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));
    m_IsSpeakingService = m_node->create_service<text_to_speech_interfaces::srv::IsSpeaking>("/TextToSpeechComponent/IsSpeaking",
                                                                                        std::bind(&TextToSpeechComponent::IsSpeaking,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));
    m_SetMicrophoneService = m_node->create_service<text_to_speech_interfaces::srv::SetMicrophone>("/TextToSpeechComponent/SetMicrophone",
                                                                                        std::bind(&TextToSpeechComponent::SetMicrophone,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));
    m_setVoiceService = m_node->create_service<text_to_speech_interfaces::srv::SetVoice>("/TextToSpeechComponent/SetVoice",
                                                                                        std::bind(&TextToSpeechComponent::SetVoice,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));


    m_speakerStatusPub = m_node->create_publisher<std_msgs::msg::Bool>("/TextToSpeechComponent/is_speaking", 10);

    m_BatchGenerationAction = rclcpp_action::create_server<text_to_speech_interfaces::action::BatchGeneration>(
        m_node,
        "/TextToSpeechComponent/BatchGenerationAction",
        std::bind(&TextToSpeechComponent::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TextToSpeechComponent::handle_cancel, this, std::placeholders::_1),
        std::bind(&TextToSpeechComponent::handle_accepted, this, std::placeholders::_1));

    m_timer = m_node->create_wall_timer(200ms,
                    [this]()->void {
			std::lock_guard<std::mutex> lock(m_mutex);
                        m_audioStatusData = m_audioStatusPort.read(false);
                        if (m_audioStatusData != nullptr)
                        {   
                            yDebugThrottle(1) << "[TextToSpeechComponent::timer] in loop reading audio status port" << __LINE__;
                            std_msgs::msg::Bool msg;
                            if(m_audioStatusData->current_buffer_size > 0)
                                msg.data = true;
                            else
                            {
                                if (!m_manualMicDisabled && m_startedSpeaking)
                                {
                                    bool isRecording;
                                    m_iAudioGrabberSound->isRecording(isRecording);
                                    if(!isRecording)
                                    {
                                        RCLCPP_ERROR_STREAM(m_node->get_logger(), "TextToSpeechComponent " << __LINE__ );
                                        if(!m_iAudioGrabberSound->startRecording())
                                        {
                                            RCLCPP_ERROR(m_node->get_logger(), "Unable to stop recording!");
                                        }
                                    }
                                    m_startedSpeaking = false;
                                }
                                msg.data = false;
                            }
                            m_speakerStatusPub->publish(msg);
                        }
                        else{
                            // RCLCPP_ERROR_STREAM(m_node->get_logger(), "TextToSpeechComponent no data received while reading audio status port" << __LINE__ );
                            yInfo() << "[TextToSpeechComponent::timer] no data received while reading audio status port";
                        }
                    });

    RCLCPP_INFO(m_node->get_logger(), "Started node");
    return true;
}

rclcpp_action::GoalResponse TextToSpeechComponent::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const text_to_speech_interfaces::action::BatchGeneration::Goal> goal)
{
    RCLCPP_INFO(m_node->get_logger(), "Received goal request with %zu texts", goal->texts.size());
    (void)uuid;
    // Let's reject sequences that are empty
    if (goal->texts.size() == 0) {
      RCLCPP_WARN(m_node->get_logger(), "Received empty sequence");
      yWarning() << "[TextToSpeechComponent::handle_goal] Received empty sequence";
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(m_node->get_logger(), "Accepted goal");
    yInfo() << "[TextToSpeechComponent::handle_goal] Accepted goal";
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TextToSpeechComponent::handle_cancel(
  const std::shared_ptr<GoalHandleBatchGeneration> goal_handle)
{
    RCLCPP_INFO(m_node->get_logger(), "Received request to cancel goal");
    yInfo() << "[TextToSpeechComponent::handle_cancel] Received request to cancel goal";
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TextToSpeechComponent::handle_accepted(const std::shared_ptr<GoalHandleBatchGeneration> goal_handle)
{
    RCLCPP_INFO(m_node->get_logger(), "Accepted goal");
    yInfo() << "[TextToSpeechComponent::handle_accepted] Accepted goal";
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread([this, goal_handle]()
    {
        this->BatchGeneration(goal_handle);
    }).detach();
}

void TextToSpeechComponent::BatchGeneration(const std::shared_ptr<GoalHandleBatchGeneration> goal_handle)
{
    RCLCPP_INFO(m_node->get_logger(), "Executing goal");
    yInfo() << "[TextToSpeechComponent::BatchGeneration] Executing goal";
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<actionBatchGeneration::Feedback>();
    auto result = std::make_shared<actionBatchGeneration::Result>();

    for (size_t i = 0; (i < goal->texts.size()) && rclcpp::ok(); ++i)
    {
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
            result->is_ok = true;
            result->error_msg = "Goal canceled";
            goal_handle->canceled(result);
            RCLCPP_INFO(m_node->get_logger(), "Goal canceled");
            yInfo() << "[TextToSpeechComponent::BatchGeneration] Goal canceled";
            return;
        }
        // Simulate work
        yarp::sig::Sound& sound = m_batchAudioPort.prepare();
        if (!m_iSpeechSynth->synthesize(goal->texts[i], sound))
        {
            yError() << "[TextToSpeechComponent::BatchGeneration] Error in synthesize";
            result->is_ok = false;
            result->error_msg = "Unable to synthesize text";
            goal_handle->abort(result);
            RCLCPP_ERROR(m_node->get_logger(), "Unable to synthesize text");
            return;
        }
        m_batchAudioPort.write();
        feedback->texts_left = goal->texts.size() - i - 1;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(m_node->get_logger(), "Published feedback for index %zu", i);
        yInfo() << "[TextToSpeechComponent::BatchGeneration] Published feedback for index " << i;
    }
    // Check if goal is done
    if (rclcpp::ok()) {
        result->is_ok = true;
        result->error_msg = "";
        goal_handle->succeed(result);
        RCLCPP_INFO(m_node->get_logger(), "Goal succeeded");
        yInfo() << "[TextToSpeechComponent::BatchGeneration] Goal succeeded";
    }
}

bool TextToSpeechComponent::close()
{
    rclcpp::shutdown();
    return true;
}

void TextToSpeechComponent::spin()
{
    rclcpp::spin(m_node);
}


void TextToSpeechComponent::Speak(const std::shared_ptr<text_to_speech_interfaces::srv::Speak::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::Speak::Response> response)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    yInfo() << "[TextToSpeechComponent::Speak] service called with text: " << request->text;
    bool isRecording;
    m_iAudioGrabberSound->isRecording(isRecording);
    if (isRecording)
    {
        m_iAudioGrabberSound->stopRecording();
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "TextToSpeechComponent stopping micorphone" << __LINE__ );
    }
    yInfo() << "[TextToSpeechComponent::Speak] passed mic";
    yarp::sig::Sound& sound = m_audioPort.prepare();
    sound.clear();
    auto init_time = yarp::os::Time::now();
    if (!m_iSpeechSynth->synthesize(request->text, sound))
    {
        yError() << "[TextToSpeechComponent::Speak] Error in synthesize";
        response->is_ok=false;
        response->error_msg="Unable to synthesize text";
        if (isRecording)
        {
            m_iAudioGrabberSound->startRecording();
            RCLCPP_ERROR_STREAM(m_node->get_logger(), "TextToSpeechComponent " << __LINE__ );
    	}
    }
    else
    {

        auto end_time =  yarp::os::Time::now();
        yInfo() << "elapsed time = " << end_time - init_time ;
        yInfo() << "[TextToSpeechComponent::Speak] synthesized with size: " << sound.getSamples();
        float speech_time = (float)(sound.getSamples()) / 44100.0f * 2; // AUDIO_BASE::rate * 2 because maybe my laptop rate is twice the one of the robot
        response->speech_time = speech_time;
        yInfo() << "[TextToSpeechComponent::Speak] speech time: " << response->speech_time;
        m_audioPort.write();
        response->is_ok=true;
    }

    
    bool isSpeaking =false;
    auto wait = 200ms;
    auto startTime = std::chrono::steady_clock::now();
    while (!isSpeaking){
        std::lock_guard<std::mutex> lock(m_mutex);
        auto data = m_audioStatusData;
        if (data != nullptr && data->current_buffer_size > 0) {
            m_startedSpeaking = true;
            isSpeaking = true;
        }
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
        if (elapsedTime > 10) // Timeout after 10 seconds
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Timeout while waiting for speech to start.");
            isSpeaking = true; // exit the loop
        }
        std::this_thread::sleep_for(wait);
    }
    yDebug() << "[TextToSpeechComponent::Speak] the robot is speaking";
}

void TextToSpeechComponent::SetLanguage(const std::shared_ptr<text_to_speech_interfaces::srv::SetLanguage::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::SetLanguage::Response> response)
{
    if (request->new_language=="")
    {
        response->is_ok=false;
        response->error_msg="Empty string passed to setting language";
    }
    else if (!m_iSpeechSynth->setLanguage(request->new_language))
    {
        response->is_ok=false;
        response->error_msg="Unable to set new language";
    }
    else
    {
        response->is_ok=true;
    }
}

void TextToSpeechComponent::SetVoice(const std::shared_ptr<text_to_speech_interfaces::srv::SetVoice::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::SetVoice::Response> response)
{
    if (request->new_voice=="")
    {
        response->is_ok=false;
        response->error_msg="Empty string passed to setting voice";
    }
    else if (!m_iSpeechSynth->setVoice(request->new_voice))
    {
        response->is_ok=false;
        response->error_msg="Unable to set new voice";
    }
    else
    {
        response->is_ok=true;
    }
}

void TextToSpeechComponent::GetLanguage(const std::shared_ptr<text_to_speech_interfaces::srv::GetLanguage::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::GetLanguage::Response> response)
{
    YARP_UNUSED(request);
    std::string current_language="";
    if (!m_iSpeechSynth->getLanguage(current_language))
    {
        response->is_ok=false;
        response->error_msg="Unable to get language from speechSynthesizer";
        response->current_language = current_language;
    }
    else
    {
        response->current_language = current_language;
        response->is_ok=true;
    }
}

void TextToSpeechComponent::IsSpeaking(const std::shared_ptr<text_to_speech_interfaces::srv::IsSpeaking::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::IsSpeaking::Response> response)
{
    YARP_UNUSED(request);
    auto timeout = 500ms;
    auto wait = 200ms;
    auto elapsed = 0ms;
    yarp::sig::AudioPlayerStatus* player_status = nullptr;

    // Read and wait untill I have a valid message, or the timeout is passed
    while ([this, &player_status]()->bool{
                std::lock_guard<std::mutex> lock(m_mutex);
                player_status = m_audioStatusData;
                if (player_status != nullptr)
                    return true;
                else
                    return false;}()
        && elapsed < timeout)
    {
        std::this_thread::sleep_for(wait);
        elapsed += wait;
    }

    if(player_status == nullptr)
    {
        response->is_ok = false;
        response->error_msg = "Timout while reading the speakers status port. No messages";
    }
    else
    {
        response->is_ok = true;
        if (player_status->current_buffer_size > 0)
        {
            response->seconds_left = player_status->current_buffer_size / 44100; //AUDIO_BASE::rate
            std::cout << "Seconds left: " << response->seconds_left << std::endl;
            response->is_speaking = true;
        }
        else
        {
            response->is_speaking = false;
        }
    }
}

void TextToSpeechComponent::SetMicrophone(const std::shared_ptr<text_to_speech_interfaces::srv::SetMicrophone::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::SetMicrophone::Response> response)
{
    bool isRecording;
    m_iAudioGrabberSound->isRecording(isRecording);
    if (request->enabled && isRecording)
    {
        response->is_ok = false;
        response->error_msg = "The robot is already Speaking";
        return;
    }

    if (request->enabled)
    {
        m_manualMicDisabled = false;
        if(!m_iAudioGrabberSound->startRecording())
        {
            response->is_ok = false;
            response->error_msg = "Unable to START recording";
        }
    }
    else
    {
        if(!m_iAudioGrabberSound->stopRecording())
        {
            response->is_ok = false;
            response->error_msg = "Unable to STOP recording";
        }
        m_manualMicDisabled = true;
    }
    response->is_ok = true;
}