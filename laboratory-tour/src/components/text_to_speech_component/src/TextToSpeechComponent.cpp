/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "yarp/sig/Sound.h"

#include "TextToSpeechComponent.hpp"

bool TextToSpeechComponent::ConfigureYARP(yarp::os::ResourceFinder &rf)
{
    bool okCheck = rf.check("SPEECHSYNTHESIZER-CLIENT");
    std::string device = "SpeechSynthesizer_nwc_yarp";
    std::string local = "/TextToSpeechComponentNode/speechClient";
    std::string remote = "/TBD/speechServer";

    if (okCheck)
    {
        yarp::os::Searchable &speech_config = rf.findGroup("SPEECHSYNTHESIZER-CLIENT");
        if (speech_config.check("device"))
        {
            device = speech_config.find("device").asString();
        }
        if (speech_config.check("local-suffix"))
        {
            local = "/TextToSpeechComponentNode" + speech_config.find("local-suffix").asString();
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
    return true;
}

bool TextToSpeechComponent::start(int argc, char*argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("TextToSpeechComponentNode");

    m_setLanguageService = m_node->create_service<text_to_speech_interfaces::srv::SetLanguage>("",
                                                                                        std::bind(&TextToSpeechComponent::SetLanguage,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));
    m_getLanguageService = m_node->create_service<text_to_speech_interfaces::srv::GetLanguage>("",
                                                                                        std::bind(&TextToSpeechComponent::GetLanguage,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));
    m_speakService = m_node->create_service<text_to_speech_interfaces::srv::Speak>("",
                                                                                        std::bind(&TextToSpeechComponent::Speak,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));

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
    if (request->text=="")
    {
        //TODO How do I stop speaking?

        response->is_ok=true;
    }
    else if (!m_iSpeechSynth->synthesize(request->text, sound))
    {
        response->is_ok=false;
        response->error_msg="Unable to synthesize text";
    }
    else
    {
        //TODO pass sound to audio device

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