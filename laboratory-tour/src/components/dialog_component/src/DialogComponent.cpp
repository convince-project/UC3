#include "DialogComponent.hpp"

#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <algorithm>

using namespace std::chrono_literals;

DialogComponent::DialogComponent() : m_random_gen(m_rand_engine()),
                                     m_uniform_distrib(1, 2)
{
    m_jsonPath = "/home/user1/UC3/laboratory-tour/conf/tours.json";
    m_tourName = "TOUR_MADAMA_3";
    m_speechTranscriberClientName = "/DialogComponent/speechTranscriberClient:i";
    m_speechTranscriberServerName = "/speechTranscription_nws/text:o";
    m_tourLoadedAtStart = false;
    m_currentPoiName = "sala_delle_guardie";
    m_exit = false;
    m_fallback_repeat_counter = 0;
    m_fallback_threshold = 3;
    m_state = IDLE;
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

    m_speechTranscriberPort.useCallback(m_speechTranscriberCallback);
    m_speechTranscriberPort.open(m_speechTranscriberClientName);
    // Try Automatic port connection
    if(! yarp::os::Network::connect(m_speechTranscriberServerName, m_speechTranscriberClientName))
    {
        yWarning() << "[DialogComponent::start] Unable to connect to: " << m_speechTranscriberServerName;
    }

    // -------------------------Speech Synthesizer nwc---------------------------------
    /*std::string device = "speechSynthesizer_nwc_yarp";
    std::string local = "/DialogComponent/speechClient";
    std::string remote = "/speechSynthesizer_nws";
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
    }*/
    std::string device, local, remote;
    // -------------------------Chat LLM nwc---------------------------------
    {
        okCheck = rf.check("POICHAT-CLIENT");
        device = "LLM_nwc_yarp";
        std::string prompt_context = "llmTest";
        std::string prompt_poi_file = "poi_madama_prompt.txt";
        std::string prompt_start_file = "Format_commands_welcome_prompt.txt";
        local = "/DialogComponent/chatBotClient/rpc:o";
        //remote = "/poi_chat/LLM_nws/rpc:i";
        remote = "/poi_madama_chat/LLM_nws/rpc:i";

        if (okCheck)
        {
            yarp::os::Searchable &chatBot_config = rf.findGroup("POICHAT-CLIENT");
            if (chatBot_config.check("prompt-context"))
            {
                prompt_context = chatBot_config.find("prompt-context").asString();
            }
            if (chatBot_config.check("prompt-poi-file"))
            {
                prompt_poi_file = chatBot_config.find("prompt-poi-file").asString();
            }
            if (chatBot_config.check("prompt-poi-file"))
            {
                prompt_poi_file = chatBot_config.find("prompt-poi-file").asString();
            }
            if (chatBot_config.check("prompt-context"))
            {
                prompt_context = chatBot_config.find("prompt-context").asString();
            }
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

        m_poiChatPoly.open(chatbot_prop);
        if (!m_poiChatPoly.isValid())
        {
            yError() << "[DialogComponent::ConfigureYARP] Error opening chatBot Client PolyDriver. Check parameters";
            return false;
        }
        m_poiChatPoly.view(m_iPoiChat);
        if (!m_iPoiChat)
        {
            yError() << "[DialogComponent::ConfigureYARP] Error opening iChatBot interface. Device not available";
            return false;
        }

        yarp::os::ResourceFinder resource_finder;
        resource_finder.setDefaultContext(prompt_context);
        std::string prompt_file_fullpath = resource_finder.findFile(prompt_poi_file);
        auto stream = std::ifstream(prompt_file_fullpath);
        if (!stream)
        {
            yWarning() << "File:" << prompt_file_fullpath << "does not exist or path is invalid";
        }
        else
        {
            std::ostringstream sstr;
            sstr << stream.rdbuf(); //Reads the entire file into the stringstream
            m_poiPrompt = sstr.str();
        }
        resource_finder.setDefaultContext(prompt_context);
        prompt_file_fullpath = resource_finder.findFile(prompt_start_file);
        stream = std::ifstream(prompt_file_fullpath);
        if (!stream)
        {
            yWarning() << "File:" << prompt_file_fullpath << "does not exist or path is invalid";
        }
        else
        {
            std::ostringstream sstr;
            sstr << stream.rdbuf(); //Reads the entire file into the stringstream
            m_startPrompt = sstr.str();
        }
    }
    // -------------------------Museum llm nwc---------------------------------
    {
        okCheck = rf.check("MUSEUMCHAT-CLIENT");
        device = "LLM_nwc_yarp";
        local = "/DialogComponent/museumConvClient/rpc:o";
        remote = "/madama_chat/LLM_nws/rpc:i";

        if (okCheck)
        {
            yarp::os::Searchable &llm_config = rf.findGroup("MUSEUMCHAT-CLIENT");
            if (llm_config.check("device"))
            {
                device = llm_config.find("device").asString();
            }
            if (llm_config.check("local-suffix"))
            {
                local = "/DialogComponent" + llm_config.find("local-suffix").asString();
            }
            if (llm_config.check("remote"))
            {
                remote = llm_config.find("remote").asString();
            }
        }

        yarp::os::Property llm_prop;
        llm_prop.put("device", device);
        llm_prop.put("local", local);
        llm_prop.put("remote", remote);

        m_museumChatPoly.open(llm_prop);
        if (!m_museumChatPoly.isValid())
        {
            yError() << "[DialogComponent::ConfigureYARP] Error opening chatBot Client PolyDriver. Check parameters";
            return false;
        }
        m_museumChatPoly.view(m_iMuseumChat);
        if (!m_iMuseumChat)
        {
            yError() << "[DialogComponent::ConfigureYARP] Error opening iChatBot interface. Device not available";
            return false;
        }
    }
    // -------------------------Generic llm nwc---------------------------------
    {
        okCheck = rf.check("GENERICCHAT-CLIENT");
        device = "LLM_nwc_yarp";
        local = "/DialogComponent/genericConvClient/rpc:o";
        remote = "/welcome_talk_chat/LLM_nws/rpc:i";

        if (okCheck)
        {
            yarp::os::Searchable &llm_config = rf.findGroup("GENERICCHAT-CLIENT");
            if (llm_config.check("device"))
            {
                device = llm_config.find("device").asString();
            }
            if (llm_config.check("local-suffix"))
            {
                local = "/DialogComponent" + llm_config.find("local-suffix").asString();
            }
            if (llm_config.check("remote"))
            {
                remote = llm_config.find("remote").asString();
            }
        }

        yarp::os::Property llm_prop;
        llm_prop.put("device", device);
        llm_prop.put("local", local);
        llm_prop.put("remote", remote);

        m_genericChatPoly.open(llm_prop);
        if (!m_genericChatPoly.isValid())
        {
            yError() << "[DialogComponent::ConfigureYARP] Error opening chatBot Client PolyDriver. Check parameters";
            return false;
        }
        m_genericChatPoly.view(m_iGenericChat);
        if (!m_iGenericChat)
        {
            yError() << "[DialogComponent::ConfigureYARP] Error opening iChatBot interface. Device not available";
            return false;
        }
    }
    // ---------------------Microphone Activation----------------------------
    /*{
        okCheck = rf.check("AUDIORECORDER-CLIENT");
        device = "audioRecorder_nwc_yarp";
        local = "/DialogComponent/audio";
        remote = "/audioRecorder_nws";

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
    }*/

    // ---------------------TOUR MANAGER-----------------------
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
	    }

            m_tourStorage = std::make_shared<TourStorage>();
            if( !m_tourStorage->LoadTour(m_jsonPath, m_tourName))
            {
                yError() << "[DialogComponent::ConfigureYARP] Unable to load tour from the given arguments: " << m_jsonPath << " and: " << m_tourName;
                return false;
            }
        }
    }

    // ---------------------SPEAKERS----------------------------
    /*{
        okCheck = rf.check("SPEAKERS");
        if (okCheck)
        {
            yarp::os::Searchable &speakersConfig = rf.findGroup("SPEAKERS");
            std::string localAudioName = "/DialogComponent/audio:o";
            std::string remoteAudioName = "/audioPlayerWrapper/audio:i";
            std::string statusRemoteName = "/audioPlayerWrapper/status:o";
            std::string statusLocalName = "/DialogComponent/audioPlayerWrapper/status:i";
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

            m_speakersAudioPort.open(localAudioName);
            if (!yarp::os::Network::connect(remoteAudioName, localAudioName))
            {
                yWarning() << "[DialogComponent::ConfigureYARP] Unable to connect port: " << remoteAudioName << " with: " << localAudioName;
            }

            m_speakersStatusPort.useCallback(m_speakerCallback);
            m_speakersStatusPort.open(statusLocalName);
            if (!yarp::os::Network::connect(statusRemoteName, statusLocalName))
            {
                yWarning() << "[DialogComponent::ConfigureYARP] Unable to connect port: " << statusRemoteName << " with: " << statusLocalName;
            }
        }
    }*/

    yInfo() << "[DialogComponent::ConfigureYARP] Successfully configured component";
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
    m_setPoiService = m_node->create_service<dialog_interfaces::srv::SetPoi>("/DialogComponent/SetPoi",
                                                                                        std::bind(&DialogComponent::SetPoi,
                                                                                            this,
                                                                                            std::placeholders::_1,
                                                                                            std::placeholders::_2));
    m_GetStateService = m_node->create_service<dialog_interfaces::srv::GetState>("/DialogComponent/GetState",
                                                                                        std::bind(&DialogComponent::GetState,
                                                                                            this,
                                                                                            std::placeholders::_1,
                                                                                            std::placeholders::_2));

    m_isSpeakingClient = m_node->create_client<text_to_speech_interfaces::srv::IsSpeaking>("/TextToSpeechComponent/IsSpeaking");
    m_setMicrophoneClient = m_node->create_client<text_to_speech_interfaces::srv::SetMicrophone>("/TextToSpeechComponent/SetMicrophone");
    m_speakClient = m_node->create_client<text_to_speech_interfaces::srv::Speak>("/TextToSpeechComponent/Speak");

    RCLCPP_INFO(m_node->get_logger(), "Started node");

    return true;
}

bool DialogComponent::close()
{
    m_state = IDLE;
    m_speechTranscriberPort.close();
    //Should I stop speaking somehow?

    //m_speakersAudioPort.close();
    //m_speakersStatusPort.close();

    if (m_dialogThread.joinable())
    {
        m_dialogThread.join();
    }

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
        // calls the scheduler to get the current poi
        std::shared_ptr<rclcpp::Node> nodeGetCurrentPoi = rclcpp::Node::make_shared("DialogComponentNodeGetCurrentPoi");
        std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::GetCurrentPoi>> clientGetCurrentPoi = nodeGetCurrentPoi->create_client<scheduler_interfaces::srv::GetCurrentPoi>("/SchedulerComponent/GetCurrentPoi");
        auto requestGetCurrentPoi = std::make_shared<scheduler_interfaces::srv::GetCurrentPoi::Request>();
        while (!clientGetCurrentPoi->wait_for_service(std::chrono::seconds(1))) {
	    std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GetCurrentPoi'. Exiting.");
                response->is_ok = false;
                response->error_msg = "Interrupted while waiting for the service 'GetCurrentPoi'. Exiting.";
                return;
            }
        }
        // send the request
        auto resultGetCurrentPoi = clientGetCurrentPoi->async_send_request(requestGetCurrentPoi);
        auto futureResultGetCurrentPoi = rclcpp::spin_until_future_complete(nodeGetCurrentPoi, resultGetCurrentPoi);
        auto responseGetCurrentPoi = resultGetCurrentPoi.get();
        if (futureResultGetCurrentPoi == rclcpp::FutureReturnCode::SUCCESS)
        {
            if( responseGetCurrentPoi->is_ok ==true) {
                m_currentPoiName = responseGetCurrentPoi->poi_name;
                // Set poi chat prompt
                if(m_currentPoiName == "madama_start")
                {
                    m_iPoiChat->deleteConversation();
                    m_iPoiChat->setPrompt(m_startPrompt);
                }
                else
                {
                    m_iPoiChat->deleteConversation();
                    m_iPoiChat->setPrompt(m_poiPrompt);
                }
            } else {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error in getting the current poi" << response->error_msg);
                response->is_ok=false;
                response->error_msg = "Error in getting the current poi" + response->error_msg;
                return;
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error in getting the current poi");
            response->is_ok=false;
            response->error_msg = "Error in getting the current poi";
            return;
        }


        // Enable mic
        /*bool recording = false;
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
        */
        m_state = RUNNING;
        // Launch thread that periodically reads the callback from the port and manages the dialog
        if (m_dialogThread.joinable())
        {
            m_dialogThread.join();
        }
        m_dialogThread = std::thread(&DialogComponent::DialogExecution, this);

        response->is_ok=true;
    }
    else
    {
        /*
        // Disable mic
        bool recording = false;
        m_iAudioGrabberSound->isRecording(recording);
        if (recording)
        {
            if (! m_iAudioGrabberSound->stopRecording())
            {
                yError() << "[DialogComponent::EnableDialog] Unable to stop recording of the mic";
                response->is_ok=false;
                return; //should we still go on? TODO
            }
        }
        */

        // kill thread and stop the speaking
        if (m_dialogThread.joinable())
        {
            m_dialogThread.join();
        }

        m_state = IDLE;
        response->is_ok=true;
    }

}

void DialogComponent::DialogExecution()
{
    long int progressive_counter = 0;
    std::chrono::duration wait_ms = 200ms;      // TODO - parameterize
    bool isRecording = false;

    // --------------------------Start Mic service call ----------------------
    {
        yInfo() << "[DialogComponent::DialogExecution] Starting Mic";
        auto setCommandClientNode = rclcpp::Node::make_shared("TextToSpeechComponentSetCommandNode");

        auto setMicrophoneClient = setCommandClientNode->create_client<text_to_speech_interfaces::srv::SetMicrophone>("/TextToSpeechComponent/SetMicrophone");
        auto request = std::make_shared<text_to_speech_interfaces::srv::SetMicrophone::Request>();
        request->enabled = true;
        // Wait for service
        while (!setMicrophoneClient->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setCommandClient'. Exiting.");
            }
        }
        auto result = setMicrophoneClient->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(setCommandClientNode, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mic Enabled");
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_microphone");
        }
    }
    // TODO check answer or error

    while (m_state!=SUCCESS)
    {
        yDebug() << "[DialogComponent::DialogExecution] iteration: " << ++progressive_counter;
        // Mic management
        //m_iAudioGrabberSound->isRecording(isRecording);
        //m_iRender->isPlaying(isPlaying);


        /*if (!isRecording && !m_speakerCallback.isPlaying())
        {
            yDebug() << "[DialogComponent::DialogExecution] Starting the recording " << __LINE__;
            m_iAudioGrabberSound->startRecording();
        }
        else if (m_speakerCallback.isPlaying())  // just for safety
        {
            yDebug() << "[DialogComponent::DialogExecution] Stopping the recording " << __LINE__;
            m_iAudioGrabberSound->stopRecording();
        }*/

        // Check if new message has been transcribed
        std::string questionText = "";
        //yInfo() << "DialogComponent::DialogExecution hasNewMessage" << __LINE__;
        if (m_speechTranscriberCallback.hasNewMessage())
        {
            if (!m_speechTranscriberCallback.getText(questionText))
            {
                std::this_thread::sleep_for(wait_ms);
                yDebug() << "[DialogComponent::DialogExecution] can't get text " << __LINE__;
                continue;
            }
        }
        else
        {
            std::this_thread::sleep_for(wait_ms);
            yDebug() << "[DialogComponent::DialogExecution] no new msg " << __LINE__;
            continue;
        }
        yInfo() << "DialogComponent::DialogExecution Getting PoIs" << __LINE__;

        // Get the poi object from the Tour manager
        PoI currentPoi;
        if(!m_tourStorage->GetTour().getPoI(m_currentPoiName, currentPoi))
        {
            yError() << "[DialogComponent::DialogExecution] Unable to get the current PoI name: " << m_currentPoiName;
            std::this_thread::sleep_for(wait_ms);
            continue;
        }
        // Generic PoI (TODO move to another place where it's done only once)
        PoI genericPoI;
        if (!m_tourStorage->GetTour().getPoI("___generic___", genericPoI))
        {
            yError() << "[DialogComponent::DialogExecution] Unable to get the generic PoI";
            std::this_thread::sleep_for(wait_ms);
            continue;
        }
        yInfo() << "DialogComponent::DialogExecution ChatBot interrogation" << __LINE__;
        // Pass the question to chatGPT
        yarp::dev::LLM_Message answer;
        ///TODO: This is an awful solution. Remove it as soon as possible
        m_lastQuestion = questionText;
        // END
        if(!m_iPoiChat->ask(questionText, answer))
        {
            yError() << "[DialogComponent::DialogExecution] Unable to interact with chatGPT with question: " << questionText;
            std::this_thread::sleep_for(wait_ms);
            continue;
        }
        std::string answerText = answer.content;
		yInfo() << "DialogComponent::DialogExecution ChatBot Output: " << answerText << __LINE__;
        // Pass the chatGPT answer to the JSON TourManager
        std::string scriptedString = "";
        if(!CommandManager(answerText, currentPoi, genericPoI, scriptedString))
        {
            yError() << "[DialogComponent::DialogExecution] Error in Command Manager for the command: " << answerText;
            std::this_thread::sleep_for(wait_ms);
            continue;
        }
    }
    return;
}

bool DialogComponent::CommandManager(const std::string &command, PoI currentPoI, PoI genericPoI, std::string & phrase)
{
    // This works for ChatGPT bot
    // We get a string formatted like in https://github.com/hsp-iit/tour-guide-robot/blob/iron/app/llmTest/conf/Format_commands_poi_prompt.txt
    // Load the command into a bottle to split it
    yarp::os::Bottle extracted_command;
    // TRIM leading and trailing white spaces ------------------------- START //
    size_t start = command.find_first_not_of(" \t\n\r");
    std::string trimmedCmd;
    if(start != std::string::npos)
    {
        size_t end = command.find_last_not_of(" \t\n\r");
        trimmedCmd = command.substr(start,end - start + 1);
    }
    // TRIM leading and trailing white spaces --------------------------- END //
    if (trimmedCmd[0] != '(')
    {
	yWarning() << "Not a bottle:" << trimmedCmd;
        yInfo() << "[DialogComponent::DialogExecution] Starting Text to Speech Service";
        auto setCommandClientNode2 = rclcpp::Node::make_shared("TextToSpeechComponentSetCommandNode2");

        auto speakClient2 = setCommandClientNode2->create_client<text_to_speech_interfaces::srv::Speak>("/TextToSpeechComponent/Speak");
        auto speak_request2 = std::make_shared<text_to_speech_interfaces::srv::Speak::Request>();
        speak_request2->text = trimmedCmd;
        // Wait for service
        while (!speakClient2->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setCommandClient'. Exiting.");
            }
        }
        auto speak_result2 = speakClient2->async_send_request(speak_request2);

        // Wait for the result.
        if (rclcpp::spin_until_future_complete(setCommandClientNode2, speak_result2) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speak succeeded");
            m_speechTranscriberCallback.setMessageConsumed();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service speak");
            m_speechTranscriberCallback.setMessageConsumed();
            return false;
        }

	    return true;
    }

    extracted_command.fromString(trimmedCmd);

    yDebug() << "[DialogComponent::InterpretCommand] Number of elements in bottle: " << extracted_command.size();
    // In the first position we have the general thing to do, like: explain, next_poi, end_tour
    auto theList = extracted_command.get(0).asList();
    yDebug() << "DialogComponent::InterpretCommand" << __LINE__;
    std::string action = theList->get(0).asString();
    yDebug() << "DialogComponent::InterpretCommand" << __LINE__;
    // yDebug() << "[DialogComponent::InterpretCommand] Got action: " << action << extracted_command.toString() << "Command: " << trimmedCmd;
    if (action == "cmd_unknown")
    {
        m_lastQuestion = theList->get(1).asString();
    }

    // Let's check what to do with the action
    if(action == "epl1")
    {
        // Check the second element for determining the exact action: artist, art_piece, historical_period, technique
        std::string topic = theList->get(1).asString();

        if (topic == "function")
        {
            action = "explainFunction";
        }
        else if (topic == "description" || topic == "descriptions" ||
                 topic == "decoration" || topic == "decorations")
        {
            action = "explainDescription";
        }
	else if (topic == "museum")
        {
            ///TODO: This is an awful solution. Remove it as soon as possible
            // Horrible solution ------------------------------------------------------------ START//

            //m_iAudioGrabberSound->stopRecording();
            std::chrono::duration wait_ms = 200ms;
            yarp::dev::LLM_Message answer;
            if(!m_iMuseumChat->ask(m_lastQuestion, answer))
            {
                yError() << "[DialogComponent::DialogExecution] Unable to interact with chatGPT with question: " << m_lastQuestion;
                std::this_thread::sleep_for(wait_ms);
            m_speechTranscriberCallback.setMessageConsumed();
                return false;
            }
            std::string answerText = answer.content;
            // ---------------------------------Text to Speech Service SPEAK------------------------------
            {
                yInfo() << "[DialogComponent::DialogExecution] Starting Text to Speech Service";
                auto setCommandClientNode = rclcpp::Node::make_shared("TextToSpeechComponentSetCommandNode");

                auto speakClient = setCommandClientNode->create_client<text_to_speech_interfaces::srv::Speak>("/TextToSpeechComponent/Speak");
                auto speak_request = std::make_shared<text_to_speech_interfaces::srv::Speak::Request>();
                speak_request->text = answerText;
                // Wait for service
                while (!speakClient->wait_for_service(std::chrono::seconds(1))) {
                    if (!rclcpp::ok()) {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setCommandClient'. Exiting.");
                    }
                }
                auto speak_result = speakClient->async_send_request(speak_request);

                // Wait for the result.
                if (rclcpp::spin_until_future_complete(setCommandClientNode, speak_result) == rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speak succeeded");
                    m_speechTranscriberCallback.setMessageConsumed();
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service speak");
                    m_speechTranscriberCallback.setMessageConsumed();
                    return false;
                }
            }

            return true;
        }
        else if(topic == "project")
        {
            ///TODO: This is an awful solution. Remove it as soon as possible
            // Horrible solution ------------------------------------------------------------ START//

            //m_iAudioGrabberSound->stopRecording();
            std::chrono::duration wait_ms = 200ms;
            yarp::dev::LLM_Message answer;
            if(!m_iGenericChat->ask(m_lastQuestion, answer))
            {
                yError() << "[DialogComponent::DialogExecution] Unable to interact with chatGPT with question: " << m_lastQuestion;
                std::this_thread::sleep_for(wait_ms);
            m_speechTranscriberCallback.setMessageConsumed();
                return false;
            }
            std::string answerText = answer.content;
            // ---------------------------------Text to Speech Service SPEAK------------------------------
            {
                yInfo() << "[DialogComponent::DialogExecution] Starting Text to Speech Service";
                auto setCommandClientNode = rclcpp::Node::make_shared("TextToSpeechComponentSetCommandNode");

                auto speakClient = setCommandClientNode->create_client<text_to_speech_interfaces::srv::Speak>("/TextToSpeechComponent/Speak");
                auto speak_request = std::make_shared<text_to_speech_interfaces::srv::Speak::Request>();
                speak_request->text = answerText;
                // Wait for service
                while (!speakClient->wait_for_service(std::chrono::seconds(1))) {
                    if (!rclcpp::ok()) {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setCommandClient'. Exiting.");
                    }
                }
                auto speak_result = speakClient->async_send_request(speak_request);

                // Wait for the result.
                if (rclcpp::spin_until_future_complete(setCommandClientNode, speak_result) == rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speak succeeded");
                    m_speechTranscriberCallback.setMessageConsumed();
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service speak");
                    m_speechTranscriberCallback.setMessageConsumed();
                    return false;
                }
            }

            return true;
        }
        else
        {
            yError() << "[DialogComponent::CommandManager] Unable to assign a known topic: " << topic;
        m_speechTranscriberCallback.setMessageConsumed();
	        return false;
        }

            /*  -------------------- COMMENTING OUT FOR CALLING SERVICE
            if (m_speakerCallback.isPlaying())
            {
                yDebug() << "[DialogComponent::InterpretCommand] Waiting for previous speech to finish" ;
            }
            while (m_speakerCallback.isPlaying() && !m_exit)
            {
                // Wait
                std::this_thread::sleep_for(100ms); // TODO - parameterize
            }

            // Synthesize the text
            yarp::sig::Sound &synthesizedSound = m_speakersAudioPort.prepare();
            synthesizedSound.clear();

            if (!m_iSpeechSynth->synthesize(answerText, synthesizedSound))
            {
                yError() << "[DialogComponent::InterpretCommand] Unable to synthesize text: " << answerText;
                return false;
            }
            yDebug() << "[DialogComponent::InterpretCommand] Have synthesized: " << answerText;
            // Close the mic
            //m_iAudioGrabberSound->stopRecording();

            yInfo() << "[DialogComponent::InterpretCommand] preparing port with duration: " << synthesizedSound.getDuration() <<  __LINE__;

            yInfo() << "[DialogComponent::InterpretCommand] Sending Sound to port" << __LINE__;
            m_speakersAudioPort.write();
            while((!m_speakerCallback.isPlaying() && m_speakerCallback.isEnabled()) && !m_exit)
            {
                std::this_thread::sleep_for(250ms);
            }
            -------------------- END COMMENTING OUT*/

        // Horrible solution -------------------------------------------------------------- END//

        // Now let's Find that trimmedCmd in the JSON
        if(! InterpretCommand(action, currentPoI, genericPoI, phrase))
        {
            yError() << "[DialogComponent::CommandManager] Interpret action: " << action;
        m_speechTranscriberCallback.setMessageConsumed();
            return false;
        }
        return true;
    }
    else if (action == "greetings")
    {
        // Now let's Find that trimmedCmd in the JSON
        yDebug() << "Got language:" << theList->get(1).asString();
        if(! InterpretCommand(action, currentPoI, genericPoI, phrase))
        {
            yError() << "[DialogComponent::CommandManager] Interpret action: " << action;
        m_speechTranscriberCallback.setMessageConsumed();
            return false;
        }
    }
    else if (action == "museum")
    {
        ///TODO: This is an awful solution. Remove it as soon as possible
        // Horrible solution ------------------------------------------------------------ START//

        //m_iAudioGrabberSound->stopRecording();
        std::chrono::duration wait_ms = 200ms;
        yarp::dev::LLM_Message answer;
        if(!m_iMuseumChat->ask(m_lastQuestion, answer))
        {
            yError() << "[DialogComponent::DialogExecution] Unable to interact with chatGPT with question: " << m_lastQuestion;
            std::this_thread::sleep_for(wait_ms);
        m_speechTranscriberCallback.setMessageConsumed();
            return false;
        }
        std::string answerText = answer.content;
        // ---------------------------------Text to Speech Service SPEAK------------------------------
        {
            yInfo() << "[DialogComponent::DialogExecution] Starting Text to Speech Service";
            auto setCommandClientNode = rclcpp::Node::make_shared("TextToSpeechComponentSetCommandNode");

            auto speakClient = setCommandClientNode->create_client<text_to_speech_interfaces::srv::Speak>("/TextToSpeechComponent/Speak");
            auto speak_request = std::make_shared<text_to_speech_interfaces::srv::Speak::Request>();
            speak_request->text = answerText;
            // Wait for service
            while (!speakClient->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setCommandClient'. Exiting.");
                }
            }
            auto speak_result = speakClient->async_send_request(speak_request);

            // Wait for the result.
            if (rclcpp::spin_until_future_complete(setCommandClientNode, speak_result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speak succeeded");
                m_speechTranscriberCallback.setMessageConsumed();
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service speak");
                m_speechTranscriberCallback.setMessageConsumed();
                return false;
            }
        }
    }
    else if (action == "general" || action == "cmd_unknown")
    {
        ///TODO: This is an awful solution. Remove it as soon as possible
        // Horrible solution ------------------------------------------------------------ START//

        //m_iAudioGrabberSound->stopRecording();
        std::chrono::duration wait_ms = 200ms;
        yarp::dev::LLM_Message answer;
        if(!m_iGenericChat->ask(m_lastQuestion, answer))
        {
            yError() << "[DialogComponent::DialogExecution] Unable to interact with chatGPT with question: " << m_lastQuestion;
            std::this_thread::sleep_for(wait_ms);
        m_speechTranscriberCallback.setMessageConsumed();
            return false;
        }
        std::string answerText = answer.content;
        // ---------------------------------Text to Speech Service SPEAK------------------------------
        {
            yInfo() << "[DialogComponent::DialogExecution] Starting Text to Speech Service";
            auto setCommandClientNode = rclcpp::Node::make_shared("TextToSpeechComponentSetCommandNode");

            auto speakClient = setCommandClientNode->create_client<text_to_speech_interfaces::srv::Speak>("/TextToSpeechComponent/Speak");
            auto speak_request = std::make_shared<text_to_speech_interfaces::srv::Speak::Request>();
            speak_request->text = answerText;
            // Wait for service
            while (!speakClient->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setCommandClient'. Exiting.");
                }
            }
            auto speak_result = speakClient->async_send_request(speak_request);

            // Wait for the result.
            if (rclcpp::spin_until_future_complete(setCommandClientNode, speak_result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speak succeeded");
                m_speechTranscriberCallback.setMessageConsumed();
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service speak");
                m_speechTranscriberCallback.setMessageConsumed();
                return false;
            }
        }
    }
    else if (action == "say")
    {
	yWarning() << "The trimmedCmd:" << trimmedCmd;
        ///TODO: This is an awful solution. Remove it as soon as possible
        // Horrible solution ------------------------------------------------------------ START//

        //m_iAudioGrabberSound->stopRecording();
        std::chrono::duration wait_ms = 200ms;
        // ---------------------------------Text to Speech Service SPEAK------------------------------
        {
            yInfo() << "[DialogComponent::DialogExecution] Starting Text to Speech Service";
            auto setCommandClientNode = rclcpp::Node::make_shared("TextToSpeechComponentSetCommandNode");

            auto speakClient = setCommandClientNode->create_client<text_to_speech_interfaces::srv::Speak>("/TextToSpeechComponent/Speak");
            auto speak_request = std::make_shared<text_to_speech_interfaces::srv::Speak::Request>();
            speak_request->text = theList->get(1).asString();;
            // Wait for service
            while (!speakClient->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setCommandClient'. Exiting.");
                }
            }
            auto speak_result = speakClient->async_send_request(speak_request);

            // Wait for the result.
            if (rclcpp::spin_until_future_complete(setCommandClientNode, speak_result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speak succeeded");
                m_speechTranscriberCallback.setMessageConsumed();
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service speak");
                m_speechTranscriberCallback.setMessageConsumed();
                return false;
            }
        }
    }
    else if (action == "next_poi" || "start_tour")   // means that it has been found // NEXT POI
    {
        m_state = SUCCESS;
        yInfo() << "[DialogComponent::InterpretCommand] Next Poi Detected" << __LINE__;
        m_speechTranscriberCallback.setMessageConsumed();
        // TODO Call a service maybe?
    }
    else if (action == "end_tour")  // END TOUR
    {
        m_state = SUCCESS;
        yInfo() << "[DialogComponent::InterpretCommand] End Tour Detected" << __LINE__;
	        m_speechTranscriberCallback.setMessageConsumed();

        // TODO call service
    } else {
	m_speechTranscriberCallback.setMessageConsumed();
	        yInfo() << "[DialogComponent::InterpretCommand] cannot interpret message " << trimmedCmd << " " << __LINE__;
    }

    return true;
}

bool DialogComponent::InterpretCommand(const std::string &command, PoI currentPoI, PoI genericPoI, std::string & phrase)
{
    bool isOk = false;
    std::vector<Action> actions;
    std::string cmd;

    bool isCurrent = currentPoI.isCommandValid(command);
    bool isGeneric = genericPoI.isCommandValid(command);
    yInfo() << __LINE__;
    if (isCurrent || isGeneric) // If the command is available either in the current PoI or the generic ones
    {
    yInfo() << __LINE__;
        int cmd_multiples;
        if (isCurrent) // If it is in the current overwrite the generic
        {
    yInfo() << __LINE__;
            cmd_multiples = currentPoI.getCommandMultiplesNum(command);
        }
        else
        {
    yInfo() << __LINE__;
            cmd_multiples = genericPoI.getCommandMultiplesNum(command);
        }

        if (cmd_multiples > 1)
        {
    yInfo() << __LINE__;
            m_uniform_distrib.param(std::uniform_int_distribution<std::mt19937::result_type>::param_type(1, cmd_multiples));
            int index = m_uniform_distrib(m_random_gen) - 1;
            cmd = command;
            if (index != 0)
            {
    yInfo() << __LINE__;
                cmd = cmd.append(std::to_string(index));
            }
        }
        else // The is only 1 command. It cannot be 0 because we checked if the command is available at the beginning
        {
    yInfo() << __LINE__;
            cmd = command;
        }

        if (isCurrent)
        {
    yInfo() << __LINE__;
            isOk = currentPoI.getActions(cmd, actions);
        }
        else
        {
    yInfo() << __LINE__;
            isOk = genericPoI.getActions(cmd, actions);
        }
    }
    else // Command is not available anywhere, return error and skip
    {
        yWarning() << "Command: " << command << " not supported in either the PoI or the generics list. Skipping...";
    }

    // After the command has been validated, let's do something
    if (isOk && !actions.empty())
    {
    yInfo() << __LINE__;
        int actionIndex = 0;
        bool isCommandBlocking = true;
        Action lastNonSignalAction;

        while (actionIndex < actions.size())
        {
            std::vector<Action> tempActions;
            for (int i = actionIndex; i < actions.size(); i++)
            {
                tempActions.push_back(actions[i]);
                if (actions[i].getType() != ActionTypes::SIGNAL)
                {
    yInfo() << __LINE__;
                    lastNonSignalAction = actions[i];
                }
                if (actions[i].isBlocking())
                {
                    actionIndex = i + 1;
    yInfo() << __LINE__;
                    break;
                }
                else
                {
                    if (i == actions.size() - 1)
                    {
    yInfo() << __LINE__;
                        actionIndex = actions.size();
                        if (!lastNonSignalAction.isBlocking())
                        {
    yInfo() << __LINE__;
                            isCommandBlocking = false;
                        }
                    }
                }
            }

            //bool containsSpeak = false;
            //float danceTime = 0.0f;

            for (Action action : tempActions) // Loops through all the actions until the blocking one. Execute all of them
            {
                switch (action.getType())
                {
                case ActionTypes::SPEAK:
                {
                    // Speak, but make it invalid if it is a fallback or it is an error message
                    //Speak(action.getParam(), (cmd != "fallback" && cmd.find("Error") == std::string::npos));
                    //phrase = action.getParam();
                    // TODO LIST:
                    //  CLOSE MIC
                    //  SYNTH TEXT
                    //  WRITE TO PORT
                    //  WAIT FOR SPEAKERS BUFFER TO BE EMPTY

                    // Check if the robot is already speaking and wait for it to finish
                    {
    yInfo() << __LINE__;
                        // waits until the robot has finished speaking
                        bool isSpeaking = false;
                        do {
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            // calls the isSpeaking service
                            auto isSpeakingClientNode = rclcpp::Node::make_shared("DialogComponentIsSpeakingNode");
                            auto isSpeakingClient = isSpeakingClientNode->create_client<text_to_speech_interfaces::srv::IsSpeaking>("/TextToSpeechComponent/IsSpeaking");
                            auto isSpeakingRequest = std::make_shared<text_to_speech_interfaces::srv::IsSpeaking::Request>();
                            while (!isSpeakingClient->wait_for_service(std::chrono::seconds(1))) {
                                if (!rclcpp::ok()) {
                                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'isSpeakingClient'. Exiting.");
                                }
                            }
                            auto isSpeakingResult = isSpeakingClient->async_send_request(isSpeakingRequest);
                            auto futureIsSpeakingResult = rclcpp::spin_until_future_complete(isSpeakingClientNode, isSpeakingResult);
                            auto isSpeakingResponse = isSpeakingResult.get();
                            isSpeaking = isSpeakingResponse->is_speaking;
    yInfo() << __LINE__;

                        } while (isSpeaking);
                    }
                    /*if (m_speakerCallback.isPlaying())
                    {
                        yDebug() << "[DialogComponent::InterpretCommand] Waiting for previous speech to finish" ;
                        while (m_speakerCallback.isPlaying() && !m_exit)
                        {
                            // Wait
                            std::this_thread::sleep_for(100ms); // TODO - parameterize
                            yDebug() << "[DialogComponent::InterpretCommand] Waiting for previous speech to finish" ;
                        }
                    }*/

                    // Synthesize the text
                    {
                        yInfo() << "[DialogComponent::DialogExecution] Starting Text to Speech Service: Speak";
                        auto setCommandClientNode = rclcpp::Node::make_shared("TextToSpeechComponentSetCommandNode");

                        auto speakClient = setCommandClientNode->create_client<text_to_speech_interfaces::srv::Speak>("/TextToSpeechComponent/Speak");
                        auto speak_request = std::make_shared<text_to_speech_interfaces::srv::Speak::Request>();
                        speak_request->text = action.getParam();
                        // Wait for service
                        while (!speakClient->wait_for_service(std::chrono::seconds(1))) {
                            if (!rclcpp::ok()) {
                                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setCommandClient'. Exiting.");
                            }
                        }
                        auto speak_result = speakClient->async_send_request(speak_request);

                        // Wait for the result.
                        if (rclcpp::spin_until_future_complete(setCommandClientNode, speak_result) == rclcpp::FutureReturnCode::SUCCESS)
                        {
                          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speak succeeded");
                          m_speechTranscriberCallback.setMessageConsumed();
                        } else {
                          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service speak");
                          m_speechTranscriberCallback.setMessageConsumed();
                          return false;
                        }
                    }
                    /*
                    yarp::sig::Sound &synthesizedSound = m_speakersAudioPort.prepare();
                    synthesizedSound.clear();
                    if (!m_iSpeechSynth->synthesize(action.getParam(), synthesizedSound))
                    {
                        yError() << "[DialogComponent::InterpretCommand] Unable to synthesize text: " << action.getParam();
                        return false;
                    }
                    yDebug() << "[DialogComponent::InterpretCommand] Have synthesized: " << action.getParam();
                    // Close the mic
                    m_iAudioGrabberSound->stopRecording();

                    yInfo() << "[DialogComponent::InterpretCommand] preparing port with duration: " << synthesizedSound.getDuration() <<  __LINE__;

                    yInfo() << "[DialogComponent::InterpretCommand] Sending Sound to port" << __LINE__;
                    m_speakersAudioPort.write();
                    */

                    //Should this be removed?   TODO - CHECK
                    //while((!m_speakerCallback.isPlaying()) && !m_exit)
                    //{
                    //    std::this_thread::sleep_for(100ms);
                    //    yDebug() << "[DialogComponent::InterpretCommand] Waiting for START playing" ;
                    //}

                    // save as backup if is a valid expression, not from an error
                    //if ((cmd != "fallback" && cmd.find("Error") == std::string::npos))
                    //{
                    //    m_last_valid_speak = phrase;
                    //}
                    //containsSpeak = true;
                    break;
                }
                case ActionTypes::DANCE:
                    // executing the command as long as I have something on the audio buffer
                    //while (m_speakerCallback.get_bufferSize() > 0)
                    //{
                        // Move around
                        // TODO
                    //}
                    yInfo() << "[DialogComponent::InterpretCommand] Dance! ";
                    break;
                case ActionTypes::SIGNAL:
                {
                    /*
                    //Signal(action.getParam());
                    bool isDelay = action.getParam().find("delay_") != std::string::npos;

                    // Patch of code to handle a delay signal blocking the parallel execution
                    if (danceTime != 0.0f && isDelay)
                    {
                        danceTime -= std::stof(action.getParam().substr(action.getParam().find("_") + 1, std::string::npos)); // Delay for the specified time in seconds.
                        if (danceTime < 0.0f)
                        {
                            danceTime = 0.0f;
                        }
                    }
                    //Check if it's already speaking
                    bool is_speaking = false;
                    if (!isSpeaking(is_speaking))
                    {
                        yError() << "Unable to execute isSpeaking().";
                    }

                    if (containsSpeak && !is_speaking && isDelay)
                    {
                        containsSpeak = false;
                    }*/
                    yInfo() << "[DialogComponent::InterpretCommand] Signal! ";
                    break;
                }
                case ActionTypes::INVALID:
                {
                    yError() << "[DialogComponent::InterpretCommand] I got an INVALID ActionType in command" << command;
                    break;
                }
                default:
                {
                    yError() << "[DialogComponent::InterpretCommand] I got an unknown ActionType.";
                    break;
                }
                }
            }

            /*if ((containsSpeak || danceTime != 0.0f) && isCommandBlocking) // Waits for the longest move in the temp list of blocked moves and speak. If there is nothing in the temp list because we are not blocking it is skipped.
            {
                //Check if it's already speaking
                bool is_speaking = false;
                if (!isSpeaking(is_speaking))
                {
                    yError() << "Unable to execute isSpeaking().";
                }
                while (containsSpeak && !is_speaking)
                {
                    if (!isSpeaking(is_speaking))
                    {
                        yError() << "Unable to execute isSpeaking()." << __LINE__;
                        break;
                    }
                    yarp::os::Time::delay(0.1);
                }
                double startTime = yarp::os::Time::now();
                while ((yarp::os::Time::now() - startTime) < danceTime)
                {
                    yarp::os::Time::delay(0.1);
                }
                while (is_speaking)
                {
                    if (!isSpeaking(is_speaking))
                    {
                        yError() << "Unable to execute isSpeaking()." << __LINE__;
                        break;
                    }
                    bool is_audio_enabled=false;
                    if (!isAudioEnabled(is_audio_enabled))
                    {
                        yError() << "Unable to execute isAudioEnabled()." << __LINE__;
                        break;
                    }
                    yarp::os::Time::delay(0.1);
                }
            }*/
        }

        /*if (cmd == "fallback")
        {
            m_fallback_repeat_counter++;
            if (m_fallback_repeat_counter == m_fallback_threshold)
            { // If the same command has been received as many times as the threshold, then repeat the question.
                //Speak(m_last_valid_speak, true);
                phrase = m_last_valid_speak;
                //BlockSpeak();
                m_fallback_repeat_counter = 0;
            }
            //Signal("startHearing"); // Open the ears after we handled the fallback to get a response.
        }
        else
        {
            m_fallback_repeat_counter = 0;
        }
        */
        return true;
    }
    return false;
}

void DialogComponent::SetLanguage(const std::shared_ptr<dialog_interfaces::srv::SetLanguage::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::SetLanguage::Response> response)
{
	yInfo() << "DialogComponent::SetLanguage call received" << __LINE__;
    if (request->new_language=="")
    {
        response->is_ok=false;
        response->error_msg="Empty string passed to setting language";
        return;
    }

    //if (!m_iSpeechSynth->setLanguage(request->new_language))
    //{
    //    response->is_ok=false;
    //    response->error_msg="Unable to set new language to speech Synth";
    //    return;
    //}
    // tourStorage -> should be loaded at start or by YARP config
    if(!m_tourStorage->m_loadedTour.setCurrentLanguage(request->new_language))
    {
        response->is_ok=false;
        response->error_msg="Unable to set new language to tourStorage";
        return;
    }
    response->is_ok=true;
}

void DialogComponent::GetLanguage([[maybe_unused]] const std::shared_ptr<dialog_interfaces::srv::GetLanguage::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::GetLanguage::Response> response)
{
    std::string current_language="";
    current_language = m_tourStorage->m_loadedTour.getCurrentLanguage();
    if (current_language!="")
    {
        response->current_language = current_language;
        response->is_ok=true;
    }
    else
    {
        response->is_ok=false;
        response->error_msg="Unable to get language from speechSynthesizer";
        response->current_language = current_language;
    }

    /*
    if (!m_tourStorage->m_loadedTour.getCurrentLanguage(current_language))
    {
        response->is_ok=false;
        response->error_msg="Unable to get language from speechSynthesizer";
        response->current_language = current_language;
    }
    else
    {
        response->current_language = current_language;
        response->is_ok=true;
    }*/
}

void DialogComponent::SetPoi(const std::shared_ptr<dialog_interfaces::srv::SetPoi::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::SetPoi::Response> response)
{
    if (request->poi_name=="")
    {
        response->is_ok=false;
        response->error_msg="Empty string passed to setPoi";
        return;
    }
    else
    {
        m_currentPoiName = request->poi_name;
        response->is_ok=true;
    }
    return;
}

void DialogComponent::GetState(const std::shared_ptr<dialog_interfaces::srv::GetState::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::GetState::Response> response)
{
    response->state = m_state;
    response->is_ok=true;
}

/*void DialogComponent::IsSpeaking(const std::shared_ptr<dialog_interfaces::srv::IsSpeaking::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::IsSpeaking::Response> response)
{
    auto timeout = 500ms;
    auto wait = 20ms;
    auto elapsed = 0ms;
    yarp::dev::AudioPlayerStatus* player_status = nullptr;

    // Read and wait untill I have a valid message, or the timeout is passed
    while ([this, &player_status]()->bool{
                player_status = m_speakersStatusPort.read();
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
}*/


/*bool DialogComponent::isSpeaking(bool &result)
{
    auto data = m_speakersStatusPort.read();
    if (data != nullptr)
    {
        yDebug() << "[DialogComponent::isSpeaking] got speakers status buffer size: " << data->current_buffer_size;
        if(data->current_buffer_size > 0)
        {
            result = true;
        }
        else
        {
            result = false;
        }
        return true;
    }
    else
    {
        yError() << "[DialogComponent::isSpeaking] unable to read from port, got a null pointer" << __LINE__;
        return false;
    }
}*/

/*bool DialogComponent::isAudioEnabled(bool &result)
{
    auto data = m_speakersStatusPort.read();
    if (data != nullptr)
    {
        yDebug() << "[DialogComponent::isAudioEnabled] got speakers audio enabled: " << data->enabled;
        result = data->enabled;
        return true;
    }
    else
    {
        yError() << "[DialogComponent::isAudioEnabled] unable to read from port, got a null pointer" << __LINE__;
        return false;
    }
}*/
