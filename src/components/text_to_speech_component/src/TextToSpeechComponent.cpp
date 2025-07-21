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
        std::string remoteAudioName = "/audioPlayerWrapper/audio:i";
        std::string statusRemoteName = "/audioPlayerWrapper/status:o";
        std::string statusLocalName = "/TextToSpeechComponent/audioPlayerWrapper/status:i";
        okCheck = rf.check("TEXT_TO_SPEECH_COMPONENT");
        if (okCheck)
        {
            yarp::os::Searchable &speakersConfig = rf.findGroup("TEXT_TO_SPEECH_COMPONENT");
            if (speakersConfig.check("localAudioName"))
            {
                localAudioName = speakersConfig.find("localAudioName").asString();
            }
            if (speakersConfig.check("remoteAudioName"))
            {
                remoteAudioName = speakersConfig.find("remoteAudioName").asString();
            }
            if (speakersConfig.check("statusRemoteName"))
            {
                statusRemoteName = speakersConfig.find("statusRemoteName").asString();
            }
            if (speakersConfig.check("statusLocalName"))
            {
                statusLocalName = speakersConfig.find("statusLocalName").asString();
            }

        }
        m_audioPort.open(localAudioName);
        if (!yarp::os::Network::connect(localAudioName, remoteAudioName))
        {
            yWarning() << "[TextToSpeechComponent::ConfigureYARP] Unable to connect port: " << remoteAudioName << " with: " << localAudioName;
        }

        m_audioStatusPort.open(statusLocalName);
        if (!yarp::os::Network::connect(statusRemoteName, statusLocalName))
        {
            yWarning() << "[TextToSpeechComponent::ConfigureYARP] Unable to connect port: " << statusRemoteName << " with: " << statusLocalName;
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

    m_timer = m_node->create_wall_timer(200ms,
                    [this]()->void {
			std::lock_guard<std::mutex> lock(m_mutex);
                        auto data = m_audioStatusPort.read();
                        if (data != nullptr)
                        {
                            std_msgs::msg::Bool msg;
                            if(data->current_buffer_size > 0)
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
                    });

    RCLCPP_INFO(m_node->get_logger(), "Started node");
    return true;
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
    while (!isSpeaking){
        auto data = m_audioStatusPort.read();
        if (data != nullptr && data->current_buffer_size > 0) {
            m_startedSpeaking = true;
            isSpeaking = true;
        }
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
    auto timeout = 500ms;
    auto wait = 20ms;
    auto elapsed = 0ms;
    yarp::sig::AudioPlayerStatus* player_status = nullptr;

    // Read and wait untill I have a valid message, or the timeout is passed
    while ([this, &player_status]()->bool{
                player_status = m_audioStatusPort.read();
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