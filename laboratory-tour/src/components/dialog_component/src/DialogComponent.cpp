#include "DialogComponent.hpp"

#include <fstream>
#include <iostream>

DialogComponent::DialogComponent()
{
    m_speechTranscriberClientName = "/DialogComponent/speechTranscriberClient:i";
    m_speechTranscriberServerName = "/speechTranscriberServer:o";
    m_tourLoadedAtStart = false;
}

bool DialogComponent::ConfigureYARP(yarp::os::ResourceFinder &rf)
{
    // -------------------------Port init----------------------------------------------
    bool okCheck = rf.check("DIALOGCOMPONENT");
    if (okCheck)
    {
        yarp::os::Searchable &component_config = rf.findGroup("DIALOGCOMPONENT");
        if (component_config.check("local-suffix"))
        {
            m_speechTranscriberClientName = "/DialogComponent" + component_config.find("local-suffix").asString();
        }
        if (component_config.check("remote-port"))
        {
            m_speechTranscriberServerName = component_config.find("remote-port").asString();
        }
    }

    // -------------------------Speech Synthesizer nwc---------------------------------
    std::string device = "speechSynthesizer_nwc_yarp";
    std::string local = "/DialogComponent/speechClient";
    std::string remote = "/speechSynthesizer/speechServer";
    {
        okCheck = rf.check("SPEECHSYNTHESIZER-CLIENT");
        if (okCheck)
        {
            yarp::os::Searchable &speech_config = rf.findGroup("SPEECHSYNTHESIZER-CLIENT");
            if (speech_config.check("device"))
            {
                device = speech_config.find("device").asString();
            }
            if (speech_config.check("local-suffix"))
            {
                local = "/DialogComponent" + speech_config.find("local-suffix").asString();
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
            yError() << "[DialogComponent::ConfigureYARP] Error opening speech synthesizer Client PolyDriver. Check parameters";
            return false;
        }
        m_speechSynthPoly.view(m_iSpeechSynth);
        if (!m_iSpeechSynth)
        {
            yError() << "[DialogComponent::ConfigureYARP] Error opening iSpeechSynth interface. Device not available";
            return false;
        }

        //Try automatic yarp port Connection
        if(! yarp::os::Network::connect(remote, local))
        {
            yWarning() << "[DialogComponent::ConfigureYARP] Unable to connect: " << local << " to: " << remote;
        }
    }
    // -------------------------Dialog Flow chatBot nwc---------------------------------
    {
        okCheck = rf.check("CHATBOT-CLIENT");
        device = "chatBot_nws_yarp";
        local = "/DialogComponent/chatBotClient";
        remote = "/dialogFlow/portname";

        if (okCheck)
        {
            yarp::os::Searchable &chatBot_config = rf.findGroup("CHATBOT-CLIENT");
            if (chatBot_config.check("device"))
            {
                device = chatBot_config.find("device").asString();
            }
            if (chatBot_config.check("local-suffix"))
            {
                local = "/DialogComponent" + chatBot_config.find("local-suffix").asString();
            }
            if (chatBot_config.check("remote"))
            {
                remote = chatBot_config.find("remote").asString();
            }
        }

        yarp::os::Property chatbot_prop;
        chatbot_prop.put("device", device);
        chatbot_prop.put("local", local);
        chatbot_prop.put("remote", remote);

        m_chatBotPoly.open(chatbot_prop);
        if (!m_chatBotPoly.isValid())
        {
            yError() << "[DialogComponent::ConfigureYARP] Error opening chatBot Client PolyDriver. Check parameters";
            return false;
        }
        m_chatBotPoly.view(m_iChatBot);
        if (!m_iChatBot)
        {
            yError() << "[DialogComponent::ConfigureYARP] Error opening iChatBot interface. Device not available";
            return false;
        }

        //Try automatic yarp port Connection
        if(! yarp::os::Network::connect(remote, local))
        {
            yWarning() << "[DialogComponent::ConfigureYARP] Unable to connect: " << local << " to: " << remote;
        }
    }
    // ---------------------Microphone Activation----------------------------
    {
        okCheck = rf.check("AUDIORECORDER-CLIENT");
        device = "audioRecorder_nwc_yarp";
        local = "/DialogComponent/audio:i";
        remote = "/audioRecorder/audio:o";

        if (okCheck)
        {
            yarp::os::Searchable &mic_config = rf.findGroup("AUDIORECORDER-CLIENT");
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

        //Try automatic yarp port Connection
        if(! yarp::os::Network::connect(remote, local))
        {
            yWarning() << "[DialogComponent::ConfigureYARP] Unable to connect: " << local << " to: " << remote;
        }
    }

    // TOUR MANAGER
    {
        if (! m_tourLoadedAtStart)
        {
            okCheck = rf.check("TOUR-MANAGER");
            if (okCheck)
            {
                yarp::os::Searchable &tour_config = rf.findGroup("TOUR-MANAGER");
                if (tour_config.check("path"))
                {
                    m_jsonPath = tour_config.find("path").asString();
                }
                if (tour_config.check("tour_name"))
                {
                    m_tourName = tour_config.find("tour_name").asString();
                }
                
                m_tourStorage = std::make_shared<TourStorage>(); 
                if( !m_tourStorage->LoadTour(m_jsonPath, m_tourName))
                {
                    yError() << "[DialogComponent::ConfigureYARP] Unable to load tour from the given arguments: " << m_jsonPath << " and: " << m_tourName;
                    return false;
                }
            }
        }
    }
    yInfo() << "[DialogComponent::ConfigureYARP] Successfully configured component";
    return true;
}

bool DialogComponent::start(int argc, char*argv[])
{
    // Loads the tour json from the file and saves a reference to the class, if the arguments are being passed at start.
    if (argc >= 2)
    {
        m_tourStorage = std::make_shared<TourStorage>(); 
        if( !m_tourStorage->LoadTour(argv[0], argv[1]))
        {
            yError() << "[DialogComponent::start] Unable to load tour from the given arguments: " << argv[0] << " and " << argv[1];
            return false;
        }
        m_tourLoadedAtStart = true;
    }

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("DialogComponentNode");

    m_setLanguageService = m_node->create_service<dialog_interfaces::srv::SetLanguage>("/DialogComponent/SetLanguage",
                                                                                        std::bind(&DialogComponent::SetLanguage,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));
    m_getLanguageService = m_node->create_service<dialog_interfaces::srv::GetLanguage>("/DialogComponent/GetLanguage",
                                                                                        std::bind(&DialogComponent::GetLanguage,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));
    m_enableDialogService = m_node->create_service<dialog_interfaces::srv::EnableDialog>("/DialogComponent/EnableDialog",
                                                                                        std::bind(&DialogComponent::EnableDialog,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));

    m_speechTranscriberPort.useCallback(m_speechTranscriberCallback);
    m_speechTranscriberPort.open(m_speechTranscriberClientName);
    // Try Automatic port connection
    if(! yarp::os::Network::connect(m_speechTranscriberServerName, m_speechTranscriberClientName))
    {
        yWarning() << "[DialogComponent::start] Unable to connect to: " << m_speechTranscriberServerName;
    }
    
    RCLCPP_INFO(m_node->get_logger(), "Started node");
    return true;
}

bool DialogComponent::close()
{
    m_speechTranscriberPort.close();
    rclcpp::shutdown();  
    return true;
}

void DialogComponent::spin()
{
    rclcpp::spin(m_node);  
}

void DialogComponent::EnableDialog(const std::shared_ptr<dialog_interfaces::srv::EnableDialog::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::EnableDialog::Response> response)
{
    if (request->enable)
    {
        // Enable mic
        bool recording = false;
        m_iAudioGrabberSound->isRecording(recording);
        if (!recording)
        {
            if (! m_iAudioGrabberSound->startRecording())
            {
                yError() << "[DialogComponent::EnableDialog] Unable to start recording of the mic";
                response->is_ok=false;
                return;
            }
        }
        
        // TODO VAD port connection ??

        // Todo launch thread that periodically reads the callback
        
        response->is_ok=true;
    }
    else
    {
        // Disable mic
        bool recording = false;
        m_iAudioGrabberSound->isRecording(recording);
        if (recording)
        {
            if (! m_iAudioGrabberSound->stopRecording())
            {
                yError() << "[DialogComponent::EnableDialog] Unable to stop recording of the mic";
                response->is_ok=false;
                return; //should we still go on?
            }
        }
        // TODO VAD port disconnection ??

        // TODO kill thread and stop the speaking
        
        response->is_ok=true;
    }
    
}

void DialogComponent::SetLanguage(const std::shared_ptr<dialog_interfaces::srv::SetLanguage::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::SetLanguage::Response> response)
{
    if (request->new_language=="")
    {
        response->is_ok=false;
        response->error_msg="Empty string passed to setting language";
        return;
    }

    if (!m_iSpeechSynth->setLanguage(request->new_language))
    {
        response->is_ok=false;
        response->error_msg="Unable to set new language to speech Synth";
        return;
    }
    if (m_iChatBot->setLanguage(request->new_language))
    {
        response->is_ok=false;
        response->error_msg="Unable to set new language";
        return;
    }
}

void DialogComponent::GetLanguage([[maybe_unused]] const std::shared_ptr<dialog_interfaces::srv::GetLanguage::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::GetLanguage::Response> response)
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
