#include "DialogComponent.hpp"

DialogComponent::DialogComponent()
{
    m_speechTranscriberClientName = "/DialogComponent/speechTranscriberClient:i";
    m_speechTranscriberServerName = "/speechTranscriberServer:o";
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
    okCheck = rf.check("SPEECHSYNTHESIZER-CLIENT");
    std::string device = "speechSynthesizer_nwc_yarp";
    std::string local = "/DialogComponent/speechClient";
    std::string remote = "/speechSynthesizer/speechServer";

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
        yError() << "Error opening speech synthesizer Client PolyDriver. Check parameters";
        return false;
    }
    m_speechSynthPoly.view(m_iSpeechSynth);
    if (!m_iSpeechSynth)
    {
        yError() << "Error opening iSpeechSynth interface. Device not available";
        return false;
    }

    // -------------------------Dialog Flow chatBot nwc---------------------------------
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
        yError() << "Error opening chatBot Client PolyDriver. Check parameters";
        return false;
    }
    m_chatBotPoly.view(m_iChatBot);
    if (!m_iChatBot)
    {
        yError() << "Error opening iChatBot interface. Device not available";
        return false;
    }

    // ---------------------Microphone Activation----------------------------

    

    //Audio Device Helper
    //m_audioDeviceHelper.ConfigureYARP(rf);
    return true;
}

bool DialogComponent::start(int argc, char*argv[])
{
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
    // Try Automatic connection
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
        // TODO enable mic

        // Todo launch thread that periodically reads the callback
        
        response->is_ok=true;
    }
    else
    {
        // TODO disable mic

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

void DialogComponent::GetLanguage(const std::shared_ptr<dialog_interfaces::srv::GetLanguage::Request> request,
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