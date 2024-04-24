/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "yarp/sig/Sound.h"

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
    //prop.put("period", 5);

    m_speechSynthPoly.open(prop);
    if (!m_speechSynthPoly.isValid())
    {
        yError() << "Error opening speech synthesizer Client PolyDriver. Check parameters";
        return false;
    }
    m_speechSynthPoly.view(m_iSpeechSynth);
    if (!m_iSpeechSynth)
    {
        yError() << "Error opening iSpeechSynth interface. Device not available";
        return false;
    }

    //Audio Device Helper
    m_audioPort.open("/TextToSpeechComponent/audio:o");
    m_audioStatusPort.open("/TextToSpeechComponent/audioStatus:i");
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
    m_speakerStatusPub = m_node->create_publisher<std_msgs::msg::Bool>("/TextToSpeechComponent/DoneSpeaking", 10);
    
    m_timer = m_node->create_wall_timer(20ms, 
                    [this]()->void {
                        auto data = m_audioStatusPort.read();
                        if (data != nullptr)
                        {
                            std_msgs::msg::Bool msg;
                            if(data->current_buffer_size > 0)
                                msg.data = true;
                            else
                                msg.data = false;
                            
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
    yarp::sig::Sound sound;
    if (!m_iSpeechSynth->synthesize(request->text, sound))
    {
        response->is_ok=false;
        response->error_msg="Unable to synthesize text";
    }
    else
    {
        auto& soundport  = m_audioPort.prepare();
        soundport = sound;
        m_audioPort.write();
        response->is_ok=true;
    }
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
    yarp::dev::AudioPlayerStatus* player_status = nullptr;

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
            response->is_speaking = true;
        else
            response->is_speaking = false;
    }
}