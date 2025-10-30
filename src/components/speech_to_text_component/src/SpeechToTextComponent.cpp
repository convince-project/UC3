/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "yarp/sig/Sound.h"

#include "SpeechToTextComponent.hpp"

using namespace std::chrono_literals;

bool SpeechToTextComponent::ConfigureYARP(yarp::os::ResourceFinder &rf)
{
    // --------------------- SPEECH TRANSCRIPTION NWC----------------------------
    bool okCheck = rf.check("SPEECHTRANSCRIPTION-CLIENT");
    std::string device = "speechTranscription_nwc_yarp";
    std::string local = "/SpeechToTextComponent/speechClient";
    std::string remote = "/speechTranscription_nws";

    if (okCheck)
    {
        yarp::os::Searchable &speech_config = rf.findGroup("SPEECHTRANSCRIPTION-CLIENT");
        if (speech_config.check("device"))
        {
            device = speech_config.find("device").asString();
        }
        if (speech_config.check("local-suffix"))
        {
            local = "/SpeechToTextComponent" + speech_config.find("local-suffix").asString();
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

    m_speechTranscrPoly.open(prop);
    if (!m_speechTranscrPoly.isValid())
    {
        yError() << "[SpeechToTextComponent::ConfigureYARP] Error opening speech synthesizer Client PolyDriver. Check parameters";
        return false;
    }
    m_speechTranscrPoly.view(m_iSpeechTranscr);
    if (!m_iSpeechTranscr)
    {
        yError() << "[SpeechToTextComponent::ConfigureYARP] Error opening iSpeechSynth interface. Device not available";
        return false;
    }

    // --------------------- Input and output ports ----------------------------
    okCheck = rf.check("SPEECH_TO_TEXT_COMPONENT");
    std::string audioInputPort = "/SpeechToTextComponent/audio:i";
    std::string transcriptionOutputPort = "/SpeechToTextComponent/text:o";

    if (okCheck)
    {
        yarp::os::Searchable &speakersConfig = rf.findGroup("SPEECH_TO_TEXT_COMPONENT");
        if (speakersConfig.check("audioInputPort"))
        {
            audioInputPort = "/SpeechToTextComponent" + speakersConfig.find("audioInputPort").asString();
        }
        if (speakersConfig.check("transcriptionOutputPort"))
        {
            transcriptionOutputPort = "/SpeechToTextComponent" + speakersConfig.find("transcriptionOutputPort").asString();
        }
    }

    if(!m_audioInputPort.open(audioInputPort))
    {
        yError() << "[SpeechToTextComponent::ConfigureYARP] Unable to open port: " << audioInputPort;
        return false;
    }
    m_audioInputPort.useCallback(*this);

    if(!m_transcriptionOutputPort.open(transcriptionOutputPort))
    {
        yError() << "[SpeechToTextComponent::ConfigureYARP] Unable to open port: " << transcriptionOutputPort;
        return false;
    }
    // --------------------- AUDIO RECORDER NWC----------------------------
    okCheck = rf.check("MICROPHONE");
    device = "audioRecorder_nwc_yarp";
    local = "/SpeechToTextComponent/audio";
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
            local = "/SpeechToTextComponent" + mic_config.find("local-suffix").asString();
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
        yError() << "[SpeechToTextComponent::ConfigureYARP] Error opening audioRecorder Client PolyDriver. Check parameters";
        return false;
    }
    m_audioRecorderPoly.view(m_iAudioGrabberSound);
    if (!m_iAudioGrabberSound)
    {
        yError() << "[SpeechToTextComponent::ConfigureYARP] Error opening audioRecorderSound interface. Device not available";
        return false;
    }

    return true;
}

bool SpeechToTextComponent::start(int argc, char*argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("SpeechToTextComponentNode");

    m_setLanguageService = m_node->create_service<text_to_speech_interfaces::srv::SetLanguage>("/SpeechToTextComponent/SetLanguage",
                                                                                        std::bind(&SpeechToTextComponent::SetLanguage,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));
    m_getLanguageService = m_node->create_service<text_to_speech_interfaces::srv::GetLanguage>("/SpeechToTextComponent/GetLanguage",
                                                                                        std::bind(&SpeechToTextComponent::GetLanguage,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));

    RCLCPP_INFO(m_node->get_logger(), "Started node");
    return true;
}

bool SpeechToTextComponent::close()
{
    rclcpp::shutdown();
    return true;
}

void SpeechToTextComponent::spin()
{
    rclcpp::spin(m_node);
}

void SpeechToTextComponent::SetLanguage(const std::shared_ptr<text_to_speech_interfaces::srv::SetLanguage::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::SetLanguage::Response> response)
{
    if (request->new_language=="")
    {
        response->is_ok=false;
        response->error_msg="Empty string passed to setting language";
        return;
    }

    auto ret = m_iSpeechTranscr->setLanguage(request->new_language);
    if (ret || ret == yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device)
    {
        response->is_ok=true;
    }
    else
    {
        response->is_ok=false;
        response->error_msg="Unable to set new language";
    }
}

void SpeechToTextComponent::GetLanguage(const std::shared_ptr<text_to_speech_interfaces::srv::GetLanguage::Request> request,
                        std::shared_ptr<text_to_speech_interfaces::srv::GetLanguage::Response> response)
{
    YARP_UNUSED(request);
    std::string current_language="";
    if (!m_iSpeechTranscr->getLanguage(current_language))
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

void SpeechToTextComponent::onRead(yarp::sig::Sound &msg)
{
    bool isRecording;
    if(!m_iAudioGrabberSound->isRecording(isRecording))
    {
        yError() << "[SpeechToTextComponent::onRead] Error checking if audio is recording";
        return;
    }
    if(isRecording)
    {
        m_iAudioGrabberSound->stopRecording();
        yInfo() << "[SpeechToTextComponent::onRead] Stopping the recording";
    }

    auto startTime = std::chrono::steady_clock::now();

    std::vector<std::string> languages = {"en-US", "it-IT", "es-ES", "fr-FR", "de-DE"};

    std::string bestTranscription;
    double bestConfidence = -1.0;
    std::string bestLanguageForTranscription = "";
    yarp::os::Bottle& outputText = m_transcriptionOutputPort.prepare();
    outputText.clear();

    if (m_iSpeechTranscr)
    {   
        std::string transcriptionText;
        double confidence;
        for (const auto &lang : languages)
        {
            auto ret = m_iSpeechTranscr->setLanguage(lang);

            if(!ret)
            {
                yError() << "[SpeechToTextComponent::onRead] Unable to set language to " << lang << ", trying next language";
                continue;
            }
            else{

                if(!m_iSpeechTranscr->transcribe(msg, transcriptionText, confidence))
                {
                    yError() << "[SpeechToTextComponent::onRead] Error transcribing audio, sending empty transcription. May the credentials be wrong?";
                    outputText.addString("");
                    outputText.addFloat64(1.0);
                    m_transcriptionOutputPort.write();
                    return;
                }
                
                if (ret == yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device)
                {   
                    // If the device does not implement language setting, this means that the transcription
                    // is indipendent from the language set, so we can break here to avoid unnecessary iterations
                    bestTranscription = transcriptionText;
                    bestConfidence = confidence;
                    break;
                }
                else
                {
                    if (confidence > bestConfidence)
                    {
                        bestConfidence = confidence;
                        bestTranscription = transcriptionText;
                        bestLanguageForTranscription = lang;
                    }
                }
            }
            
        }

        yInfo() << "[SpeechToTextComponent::onRead] Transcription: " << bestTranscription << " with confidence: " << bestConfidence;
        if (bestLanguageForTranscription != "")
        {
            yInfo() << "[SpeechToTextComponent::onRead] Language used for transcription: " << bestLanguageForTranscription;
        }
        outputText.addString(bestTranscription);
        outputText.addFloat64(bestConfidence);
        m_transcriptionOutputPort.write();
    }
    else
    {
        yError() << "[SpeechToTextComponent::onRead] Error opening iSpeechSynth interface. Device not available";
    }

    auto currentTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
    std::cout << "Elapsed time: " << elapsedTime << " seconds" << std::endl;
}
