#include "DialogComponent.hpp"

#include <fstream>
#include <iostream>
#include <random>
#include <string>

using namespace std::chrono_literals;

DialogComponent::DialogComponent() : m_random_gen(m_rand_engine()),
                                     m_uniform_distrib(1, 2)
{
    m_speechTranscriberClientName = "/DialogComponent/speechTranscriberClient:i";
    m_speechTranscriberServerName = "/speechTranscription_nws/text:o";
    m_tourLoadedAtStart = false;
    m_currentPoiName = "gam_uccello";
    m_exit = false;
    m_fallback_repeat_counter = 0;
    m_fallback_threshold = 3;
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

        m_speechTranscriberPort.useCallback(m_speechTranscriberCallback);
        m_speechTranscriberPort.open(m_speechTranscriberClientName);
        // Try Automatic port connection
        if(! yarp::os::Network::connect(m_speechTranscriberServerName, m_speechTranscriberClientName))
        {
            yWarning() << "[DialogComponent::start] Unable to connect to: " << m_speechTranscriberServerName;
        }
    }

    // -------------------------Speech Synthesizer nwc---------------------------------
    std::string device = "speechSynthesizer_nwc_yarp";
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

        //Try automatic yarp port Connection
        //if(! yarp::os::Network::connect(remote, local))
        //{
        //    yWarning() << "[DialogComponent::ConfigureYARP] Unable to connect: " << local << " to: " << remote;
        //}
    }
    // -------------------------Dialog Flow chatBot nwc---------------------------------
    {
        okCheck = rf.check("CHATBOT-CLIENT");
        //device = "chatBot_nwc_yarp";
        //local = "/DialogComponent/chatBotClient";
        //remote = "/chatBot_nws";
        device = "LLM_nwc_yarp";
        local = "/DialogComponent/chatBotClient";
        remote = "/poi_chat/LLM_nws/rpc";

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

        m_chatGPTPoly.open(chatbot_prop);
        if (!m_chatGPTPoly.isValid())
        {
            yError() << "[DialogComponent::ConfigureYARP] Error opening chatBot Client PolyDriver. Check parameters";
            return false;
        }
        m_chatGPTPoly.view(m_iChatGPT);
        if (!m_iChatGPT)
        {
            yError() << "[DialogComponent::ConfigureYARP] Error opening iChatBot interface. Device not available";
            return false;
        }
        //m_chatBotPoly.open(chatbot_prop);
        //if (!m_chatBotPoly.isValid())
        //{
        //    yError() << "[DialogComponent::ConfigureYARP] Error opening chatBot Client PolyDriver. Check parameters";
        //    return false;
        //}
        //m_chatBotPoly.view(m_iChatBot);
        //if (!m_iChatBot)
        //{
        //    yError() << "[DialogComponent::ConfigureYARP] Error opening iChatBot interface. Device not available";
        //    return false;
        //}
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
        //if(! yarp::os::Network::connect(remote, local))
        //{
        //    yWarning() << "[DialogComponent::ConfigureYARP] Unable to connect: " << local << " to: " << remote;
        //}
    }

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
                
                m_tourStorage = std::make_shared<TourStorage>(); 
                if( !m_tourStorage->LoadTour(m_jsonPath, m_tourName))
                {
                    yError() << "[DialogComponent::ConfigureYARP] Unable to load tour from the given arguments: " << m_jsonPath << " and: " << m_tourName;
                    return false;
                }
            }
        }
    }

    // ---------------------SPEAKERS----------------------------
    {
        okCheck = rf.check("SPEAKERS");
        if (okCheck)
        {
            yarp::os::Searchable &speakersConfig = rf.findGroup("SPEAKERS");
            std::string localAudioName = "/DialogComponent/audio:o";
            std::string remoteAudioName = "/audioPlayerWrapper:i";
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

            m_speakersStatusPort.open(statusLocalName);
            if (!yarp::os::Network::connect(statusRemoteName, statusLocalName))
            {
                yWarning() << "[DialogComponent::ConfigureYARP] Unable to connect port: " << statusRemoteName << " with: " << statusLocalName;
            }
        }
    }

    yInfo() << "[DialogComponent::ConfigureYARP] Successfully configured component";
    return true;
}

bool DialogComponent::start(int argc, char*argv[])
{
    // Loads the tour json from the file and saves a reference to the class, if the arguments are being passed at start.
    //if (argc >= 2)
    //{
    //    m_tourStorage = std::make_shared<TourStorage>(); 
    //    if( !m_tourStorage->LoadTour(argv[0], argv[1]))
    //    {
    //        yError() << "[DialogComponent::start] Unable to load tour from the given arguments: " << argv[0] << " and " << argv[1];
    //        return false;
    //    }
    //    m_tourLoadedAtStart = true;
    //}

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
    
    RCLCPP_INFO(m_node->get_logger(), "Started node");
    return true;
}

bool DialogComponent::close()
{
	m_exit = true;
    m_speechTranscriberPort.close();
    //Should I stop speaking somehow?

    m_speakersAudioPort.close();
    m_speakersStatusPort.close();
    
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

        // TODO kill thread and stop the speaking
        if (m_dialogThread.joinable())
        {
            m_dialogThread.join();
        }

        response->is_ok=true;
    }
    
}

void DialogComponent::DialogExecution()
{
    std::chrono::duration wait_ms = 200ms;
    while (!m_exit)
    {
		//yInfo() << "DialogComponent::DialogExecution call received" << __LINE__;
        // Mic management
        bool isRecording = false;
        bool isPlaying = false;
        m_iAudioGrabberSound->isRecording(isRecording);
        //m_iRender->isPlaying(isPlaying);
        yarp::dev::AudioPlayerStatus *playerStatus = m_speakersStatusPort.read();
        if (playerStatus==nullptr)
        {
            yError() << "[DialogComponent::DialogExecution] Unable to get AudioPlayerStatus";
            std::this_thread::sleep_for(wait_ms);
            continue;
        }
        if (playerStatus->current_buffer_size > 0)
        {
            isPlaying = true;
        }
        
        if (!isRecording && !isPlaying)
        {
            m_iAudioGrabberSound->startRecording();
        }
        // Check if new message has been transcribed
        std::string questionText = "";
        //yInfo() << "DialogComponent::DialogExecution hasNewMessage" << __LINE__;
        if (m_speechTranscriberCallback.hasNewMessage())
        {
            if (!m_speechTranscriberCallback.getText(questionText))
            {
                std::this_thread::sleep_for(wait_ms);
                continue;
            }
        }
        else
        {
            std::this_thread::sleep_for(wait_ms);
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
        // Pass the question to DialogFlow
        yarp::dev::LLM_Message answer;
        if(!m_iChatGPT->ask(questionText, answer))
        {
            yError() << "[DialogComponent::DialogExecution] Unable to interact with chatGPT with question: " << questionText;
            std::this_thread::sleep_for(wait_ms);
            continue;
        }
        std::string answerText = answer.content;
		yInfo() << "DialogComponent::DialogExecution ChatBot Output: " << answerText << __LINE__;
        // Pass the DialogFlow answer to the JSON TourManager
        std::string scriptedString = "";
        if(!CommandManager(answerText, currentPoi, genericPoI, scriptedString))
        {
            yError() << "[DialogComponent::DialogExecution] Unable to interpret command: " << answerText;
            std::this_thread::sleep_for(wait_ms);
            continue;
        }
		yInfo() << "DialogComponent::DialogExecution Interpreted command: " << scriptedString << __LINE__;
        // Synthetise the text
        yarp::sig::Sound synthesizedSound;
        if (!m_iSpeechSynth->synthesize(scriptedString, synthesizedSound))
        {
            yError() << "[DialogComponent::DialogExecution] Unable to synthesize text: " << scriptedString;
            std::this_thread::sleep_for(wait_ms);
            continue;
        }
		yInfo() << "DialogComponent::DialogExecution Stopping recording" << __LINE__;
        m_iAudioGrabberSound->stopRecording();  //Do I need to stop recording?
        // Pass the sound to the speaker -> Do I have to shut down also the mic ?? TODO
        yInfo() << "DialogComponent::DialogExecution preparing port with duration: " << synthesizedSound.getDuration() <<  __LINE__;
        m_speakersAudioPort.prepare() = synthesizedSound;
        //auto & buffer = m_speakersAudioPort.prepare();
        //buffer.clear();
        //buffer = synthesizedSound;
        yInfo() << "DialogComponent::DialogExecution Sending Sound to port" << __LINE__;
        m_speakersAudioPort.write();
        std::this_thread::sleep_for(wait_ms);
    }
    return;
}

bool DialogComponent::CommandManager(const std::string &command, PoI currentPoI, PoI genericPoI, std::string & phrase)
{
    // This works for ChatGPT bot
    // We get a string formatted like in https://github.com/hsp-iit/tour-guide-robot/blob/iron/app/llmTest/conf/Format_commands_poi_prompt.txt
    // Load the command into a bottle to split it
    //yarp::os::Bottle extracted_command;
    //extracted_command.fromString(command);
    std::string replaced_str = command;
    std::string action;
	// Find what to say:
	auto first_pos = replaced_str.find('"');
	if (first_pos != replaced_str.size())
    {
		auto second_pos = replaced_str.find('"', first_pos + 1);
        yInfo() << "[DialogComponent::CommandManager] begin: " << first_pos << " end: " << second_pos;
		if (second_pos != replaced_str.size())
		{
            action = replaced_str.substr(first_pos + 1, (second_pos - first_pos) - 1);
            //yInfo() << "[DialogComponent::InterpretCommand] returning from the raw say: " << phrase << __LINE__;
			//auto third_pos = replaced_str.find('"', second_pos + 1);
		}
	}
	
    //yDebug() << "[DialogComponent::InterpretCommand] Number of elements in bottle: " << extracted_command.size();
    // In the first position we have the general thing to do, like: explain, next_poi, end_tour
    //std::string action = extracted_command.get(0).asString();
    yDebug() << "[DialogComponent::InterpretCommand] Got action: " << action;
    // Let's check what to do with the action
	// Find what to say:
	auto explain_pos = command.find("explain");
    if (explain_pos < command.size())    //found
    {
        // TODO create a struct map
        if (command.find("artist") < command.size())
        {
            action = "explainQuestionAuthor";
        }
        else if (command.find("art_piece") < command.size())
        {
            action = "explainQuestionMainPiece";
        }
        else if (command.find("historical_period") < command.size())
        {
            action = "explainQuestionEpoch";
        }
        else if (command.find("technique") < command.size())
        {
            action = "explainQuestionTechnique";
        }
        else
        {
            yError() << "[DialogComponent::CommandManager] Unable to assign a known topic";
	        return false;
        }
        yDebug() << "[DialogComponent::InterpretCommand] Got topic: " << action;

        // TODO for other action types
        // Check the second element for determining the exact action: artist, art_piece, historical_period, technique
        //std::string topic = extracted_command.get(1).asString();
        /*if (topic == "artist")
        {
            action = "explainQuestionAuthor";
        }
        else if (topic == "art_piece")
        {
            action = "explainQuestionMainPiece";
        }
        else if (topic == "historical_period")
        {
            action = "explainQuestionEpoch";
        }
        else if (topic == "technique")
        {
            action = "explainQuestionTechnique";
        }
        else
        {
            yError() << "[DialogComponent::CommandManager] Unable to assign a known topic: " << topic;
	        return false;
        }*/

        // Now let's Find that command in the JSON
        if(! InterpretCommand(action, currentPoI, genericPoI, phrase))
        {
            yError() << "[DialogComponent::CommandManager] Interpret action: " << action;
            return false;
        }
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

    if (isCurrent || isGeneric) // If the command is available either in the current PoI or the generic ones
    {
        int cmd_multiples;
        if (isCurrent) // If it is in the current overwrite the generic
        {
            cmd_multiples = currentPoI.getCommandMultiplesNum(command);
        }
        else
        {
            cmd_multiples = genericPoI.getCommandMultiplesNum(command);
        }

        if (cmd_multiples > 1)
        {
            m_uniform_distrib.param(std::uniform_int_distribution<std::mt19937::result_type>::param_type(1, cmd_multiples));
            int index = m_uniform_distrib(m_random_gen) - 1;
            cmd = command;
            if (index != 0)
            {
                cmd = cmd.append(std::to_string(index));
            }
        }
        else // The is only 1 command. It cannot be 0 because we checked if the command is available at the beginning
        {
            cmd = command;
        }

        if (isCurrent)
        {
            isOk = currentPoI.getActions(cmd, actions);
        }
        else
        {
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
                    lastNonSignalAction = actions[i];
                }
                if (actions[i].isBlocking())
                {
                    actionIndex = i + 1;
                    break;
                }
                else
                {
                    if (i == actions.size() - 1)
                    {
                        actionIndex = actions.size();
                        if (!lastNonSignalAction.isBlocking())
                        {
                            isCommandBlocking = false;
                        }
                    }
                }
            }

            bool containsSpeak = false;
            float danceTime = 0.0f;

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

                    // save as backup if is a valid expression, not from an error
                    if ((cmd != "fallback" && cmd.find("Error") == std::string::npos))
                    {
                        m_last_valid_speak = phrase;
                    }
                    containsSpeak = true;
                    break;
                }
                case ActionTypes::DANCE:
                    break;
                case ActionTypes::SIGNAL:
                {
                    /*
                    //Signal(action.getParam());    // TODO
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
                    break;
                }
                case ActionTypes::INVALID:
                {
                    yError() << "I got an INVALID ActionType in command" << command;
                    break;
                }
                default:
                {
                    yError() << "I got an unknown ActionType.";
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

    if (!m_iSpeechSynth->setLanguage(request->new_language))
    {
        response->is_ok=false;
        response->error_msg="Unable to set new language to speech Synth";
        return;
    }
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

void DialogComponent::SetPoi(const std::shared_ptr<dialog_interfaces::srv::SetPoi::Request> request,
                        std::shared_ptr<dialog_interfaces::srv::SetPoi::Response> response)
{
    if (request->poi_name=="")
    {
        response->is_ok=false;
        response->error_msg="Empty string passed to setting language";
        return;
    }
    else
    {
        m_currentPoiName = request->poi_name;
        response->is_ok=true;
    }
    return;
}

bool DialogComponent::isSpeaking(bool &result)
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
}

bool DialogComponent::isAudioEnabled(bool &result)
{
    auto data = m_speakersStatusPort.read();
    if (data != nullptr)
    {
        yDebug() << "[DialogComponent::isSpeaking] got speakers audio enabled: " << data->enabled;
        result = data->enabled;
        return true;
    }
    else
    {
        yError() << "[DialogComponent::isSpeaking] unable to read from port, got a null pointer" << __LINE__;
        return false;
    }
}