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
    m_jsonPath = "/home/user1/UC3/conf/tours-with-italian-dates-in-chars.json";
    m_tourName = "TOUR_MADAMA_3";
    m_speechToTextClientName = "/DialogComponent/SpeechToTextClient:i";
    m_speechToTextServerName = "/SpeechToTextComponent/text:o";
    m_tourLoadedAtStart = false;
    m_currentPoiName = "sala_delle_guardie";
    // m_exit = false
    m_fallback_repeat_counter = 0;
    m_fallback_threshold = 3;
    m_state = IDLE;
    m_predefined_answer_index = 0;
    m_predefined_answer = "";

    // m_duplicateIndex = -1;
    m_last_received_interaction = "";

    m_number_of_predefined_answers = 0;
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
            m_speechToTextClientName = "/DialogComponent" + component_config.find("local-suffix").asString();
        }
        if (component_config.check("remote-port"))
        {
            m_speechToTextServerName = component_config.find("remote-port").asString();
        }
    }

    m_speechToTextPort.open(m_speechToTextClientName);
    // Try Automatic port connection
    if (!yarp::os::Network::connect(m_speechToTextServerName, m_speechToTextClientName))
    {
        yWarning() << "[DialogComponent::start] Unable to connect to: " << m_speechToTextServerName;
    }

    yDebug() << "[DialogComponent::start] Remote Port: " << m_speechToTextServerName;
    yDebug() << "[DialogComponent::start] Local Port: " << m_speechToTextClientName;

    std::string device, local, remote;
    // -------------------------Chat LLM nwc---------------------------------
    {
        okCheck = rf.check("POICHAT-CLIENT");
        device = "LLM_nwc_yarp";
        std::string prompt_context = "convince";
        std::string prompt_poi_file = "poi_madama_prompt.txt";
        std::string prompt_start_file = "Format_commands_welcome_prompt.txt";
        local = "/DialogComponent/chatBotClient/rpc:o";
        // remote = "/poi_chat/LLM_nws/rpc:i";
        remote = "/welcome_chat/LLM_nws/rpc:i";

        if (okCheck)
        {
            yarp::os::Searchable &chatBot_config = rf.findGroup("POICHAT-CLIENT");
            // Why are these two lines repeated?
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
        std::cout << "Prompt file full path: " << prompt_file_fullpath << std::endl;
        auto stream = std::ifstream(prompt_file_fullpath);
        if (!stream)
        {
            yWarning() << "File:" << prompt_file_fullpath << "does not exist or path is invalid";
        }
        else
        {
            std::ostringstream sstr;
            sstr << stream.rdbuf(); // Reads the entire file into the stringstream
            m_poiPrompt = sstr.str();
        }
        resource_finder.setDefaultContext(prompt_context);
        prompt_file_fullpath = resource_finder.findFile(prompt_start_file);
        std::cout << "Start Prompt file full path: " << prompt_file_fullpath << std::endl;
        stream = std::ifstream(prompt_file_fullpath);
        if (!stream)
        {
            yWarning() << "File:" << prompt_file_fullpath << "does not exist or path is invalid";
        }
        else
        {
            std::ostringstream sstr;
            sstr << stream.rdbuf(); // Reads the entire file into the stringstream
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

    // ----- VOICES ---------
    {
        std::string eng = "en-US-Standard-G";
        std::string ita = "it-IT-Standard-A";
        std::string fra = "fr-FR-Standard-A";
        std::string ger = "de-DE-Standard-A";
        std::string spa = "es-ES-Standard-C";
        std::string jap = "ja-JP-Standard-B";

        if (rf.check("VOICES"))
        {
            yarp::os::Searchable &voices_cfg = rf.findGroup("VOICES");
            if (voices_cfg.check("it-IT"))
            {
                ita = voices_cfg.find("it-IT").asString();
            }
            if (voices_cfg.check("en-US"))
            {
                eng = voices_cfg.find("en-US").asString();
            }
            if (voices_cfg.check("fr-FR"))
            {
                fra = voices_cfg.find("fr-FR").asString();
            }
            if (voices_cfg.check("de-DE"))
            {
                ger = voices_cfg.find("de-DE").asString();
            }
            if (voices_cfg.check("es-ES"))
            {
                spa = voices_cfg.find("es-ES").asString();
            }
            if (voices_cfg.check("ja-JP"))
            {
                jap = voices_cfg.find("ja-JP").asString();
            }
        }
        m_voicesMap.clear();
        m_voicesMap["it-IT"] = ita;
        m_voicesMap["en-US"] = eng;
        m_voicesMap["fr-FR"] = fra;
        m_voicesMap["de-DE"] = ger;
        m_voicesMap["es-ES"] = spa;
        m_voicesMap["ja-JP"] = jap;
    }

    // ---------------------TOUR MANAGER-----------------------
    {
        if (!m_tourLoadedAtStart)
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

            yInfo() << "[DialogComponent::ConfigureYARP] Loading tour from: " << m_jsonPath << " and: " << m_tourName;

            m_tourStorage = std::make_shared<TourStorage>();
            if (!m_tourStorage->LoadTour(m_jsonPath, m_tourName))
            {
                yError() << "[DialogComponent::ConfigureYARP] Unable to load tour from the given arguments: " << m_jsonPath << " and: " << m_tourName;
                return false;
            }
        }
    }

    // ---------------------SPEAKERS----------------------------
    {
        std::string localAudioName = "/DialogComponent/audio:o";
        okCheck = rf.check("DIALOGCOMPONENT");
        if (okCheck)
        {
            yarp::os::Searchable &speakersConfig = rf.findGroup("DIALOGCOMPONENT");
            if (speakersConfig.check("localAudioName"))
            {
                localAudioName = speakersConfig.find("localAudioName").asString();
            }
        }

        if (!m_audioPort.open(localAudioName))
        {
            yError() << "[TextToSpeechComponent::ConfigureYARP] Unable to open port: " << localAudioName;
            return false;
        }
    }

    if (!m_verbalOutputBatchReader.ConfigureYARP(rf))
    {
        yError() << "[DialogComponent::ConfigureYARP] Unable to configure VerbalOutputBatchReader";
        return false;
    }

    yInfo() << "[DialogComponent::ConfigureYARP] Successfully configured component";


    // set face expression port
    m_faceexpression_rpc_port_name = "/DialogComponent/FaceExpressionClient/rpc:o";
    if (!m_faceexpression_rpc_port.open(m_faceexpression_rpc_port_name))
    {
        yError() << "[DialogComponent::ConfigureYARP] Unable to open Face Expression RPC port: " << m_faceexpression_rpc_port_name;
        return false;
    }


    return true;
}

bool DialogComponent::start(int argc, char *argv[])
{
    if (!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }

    m_node = rclcpp::Node::make_shared("DialogComponentNode");


    nodeGetCurrentPoi = rclcpp::Node::make_shared("DialogComponentNodeGetCurrentPoi");
    clientGetCurrentPoi = nodeGetCurrentPoi->create_client<scheduler_interfaces::srv::GetCurrentPoi>("/SchedulerComponent/GetCurrentPoi");

    setLangClientNode = rclcpp::Node::make_shared("DialogComponentSetLangNode");
    setLangClient = setLangClientNode->create_client<scheduler_interfaces::srv::SetLanguage>("/SchedulerComponent/SetLanguage");

    setLangClientNode2 = rclcpp::Node::make_shared("DialogComponentSetLangNode2");
    setLangClient2 = setLangClientNode2->create_client<text_to_speech_interfaces::srv::SetLanguage>("/TextToSpeechComponent/SetLanguage");

    setVoiceClientNode = rclcpp::Node::make_shared("DialogComponentSetVoiceNode");
    setVoiceClient = setVoiceClientNode->create_client<text_to_speech_interfaces::srv::SetVoice>("/TextToSpeechComponent/SetVoice");

    setSpeechToTextLanguageClientNode = rclcpp::Node::make_shared("DialogComponentSetSpeechToTextLanguageNode");
    setSpeechToTextLanguageClient = setSpeechToTextLanguageClientNode->create_client<text_to_speech_interfaces::srv::SetLanguage>("/SpeechToTextComponent/SetLanguage");

    isSpeakingClientNode = rclcpp::Node::make_shared("DialogComponentIsSpeakingNode");
    isSpeakingClient = isSpeakingClientNode->create_client<text_to_speech_interfaces::srv::IsSpeaking>("/TextToSpeechComponent/IsSpeaking");

    executeDanceClientNode = rclcpp::Node::make_shared("ExecuteDanceComponentExecuteDanceNode");
    danceClient = executeDanceClientNode->create_client<execute_dance_interfaces::srv::ExecuteDance>("/ExecuteDanceComponent/ExecuteDance");

    executePointingClientNode = rclcpp::Node::make_shared("CartesianPointingComponentPointTaskNode");
    pointingClient = executePointingClientNode->create_client<cartesian_pointing_interfaces::srv::PointAt>("/CartesianPointingComponent/PointAt");

    isMotionDoneClientNode = rclcpp::Node::make_shared("DialogComponentMotionDoneNode");
    isMotionDoneClient = isMotionDoneClientNode->create_client<cartesian_pointing_interfaces::srv::IsMotionDone>("/CartesianPointingComponent/IsMotionDone");

    m_manageContextService = m_node->create_service<dialog_interfaces::srv::ManageContext>("/DialogComponent/ManageContext",
                                                                                           std::bind(&DialogComponent::ManageContext,
                                                                                                     this,
                                                                                                     std::placeholders::_1,
                                                                                                     std::placeholders::_2));

    m_ShortenReplyService = m_node->create_service<dialog_interfaces::srv::ShortenReply>("/DialogComponent/ShortenReply",
                                                                                         std::bind(&DialogComponent::ShortenReply,
                                                                                                   this,
                                                                                                   std::placeholders::_1,
                                                                                                   std::placeholders::_2));

    m_AnswerService = m_node->create_service<dialog_interfaces::srv::Answer>("/DialogComponent/Answer",
                                                                             std::bind(&DialogComponent::Answer,
                                                                                       this,
                                                                                       std::placeholders::_1,
                                                                                       std::placeholders::_2));

    m_SetLanguageService = m_node->create_service<dialog_interfaces::srv::SetLanguage>("/DialogComponent/SetLanguage",
                                                                                       std::bind(&DialogComponent::SetLanguage,
                                                                                                 this,
                                                                                                 std::placeholders::_1,
                                                                                                 std::placeholders::_2));

    m_InterpretCommandService = m_node->create_service<dialog_interfaces::srv::InterpretCommand>("/DialogComponent/InterpretCommand",
                                                                                                 std::bind(&DialogComponent::InterpretCommand,
                                                                                                           this,
                                                                                                           std::placeholders::_1,
                                                                                                           std::placeholders::_2));

    m_WaitForInteractionAction = rclcpp_action::create_server<dialog_interfaces::action::WaitForInteraction>(
        m_node,
        "/DialogComponent/WaitForInteractionAction",
        std::bind(&DialogComponent::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DialogComponent::handle_cancel, this, std::placeholders::_1),
        std::bind(&DialogComponent::handle_accepted, this, std::placeholders::_1));

    m_SpeakAction = rclcpp_action::create_server<dialog_interfaces::action::Speak>(
        m_node,
        "/DialogComponent/SpeakAction",
        std::bind(&DialogComponent::handle_speak_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DialogComponent::handle_speak_cancel, this, std::placeholders::_1),
        std::bind(&DialogComponent::handle_speak_accepted, this, std::placeholders::_1));

    if (!UpdatePoILLMPrompt())
    {
        yError() << "[DialogComponent::ConfigureYarp] Error in UpdatePoILLMPrompt";
        return false;
    }

    RCLCPP_INFO(m_node->get_logger(), "Started node");

    return true;
}

bool DialogComponent::close()
{
    m_state = IDLE;
    m_speechToTextPort.close();
    // Should I stop speaking somehow?

    rclcpp::shutdown();
    return true;
}

void DialogComponent::spin()
{
    rclcpp::spin(m_node);
}

rclcpp::Node::SharedPtr DialogComponent::getNode()
{
    return m_node;
}

void DialogComponent::ManageContext(const std::shared_ptr<dialog_interfaces::srv::ManageContext::Request> request,
                                    std::shared_ptr<dialog_interfaces::srv::ManageContext::Response> response)
{

    yInfo() << "DialogComponent::ManageContext ChatBot interrogation" << __LINE__;
    // Pass the question to chatGPT
    yarp::dev::LLM_Message answer;
    /// TODO: This is an awful solution. Remove it as soon as possible
    // END

    std::chrono::duration wait_ms = 200ms;
    if (!m_iPoiChat->ask(m_last_received_interaction, answer))
    {
        yError() << "[DialogComponent::ManageContext] Unable to interact with chatGPT with question: " << m_last_received_interaction;
    }
    std::string answerText = answer.content;
    yInfo() << "[DialogComponent::ManageContext] ChatBot Output: " << answerText << __LINE__;
    // Pass the chatGPT answer to the JSON TourManager

    if (!CommandManager(answerText, response))
    {
        yError() << "[DialogComponent::ManageContext] Error in Command Manager for the command: " << answerText;
    }

    yDebug() << "[DialogComponent::ManageContext] Response from Command Manager: " << response->is_ok << response->language << response->context;

    return;
}

bool DialogComponent::CommandManager(const std::string &command, std::shared_ptr<dialog_interfaces::srv::ManageContext::Response> &response)
{
    // set the end of the tour to false, change it only if we enter the action end_tour
    response->is_poi_ended = false;
    // This works for ChatGPT bot
    // We get a string formatted like in https://github.com/hsp-iit/tour-guide-robot/blob/iron/app/llmTest/conf/Format_commands_poi_prompt.txt
    // Load the command into a bottle to split it
    yarp::os::Bottle extracted_command;
    // TRIM leading and trailing white spaces ------------------------- START //
    size_t start = command.find_first_not_of(" \t\n\r");
    std::string trimmedCmd;
    if (start != std::string::npos)
    {
        size_t end = command.find_last_not_of(" \t\n\r");
        trimmedCmd = command.substr(start, end - start + 1);
    }
    // TRIM leading and trailing white spaces --------------------------- END //
    // If trimmedCmd does not start with a '(', it means that the command is not a bottle, therefore we can reply to the user with the previously generated text
    if (trimmedCmd[0] != '(')
    {
        yWarning() << "Not a bottle:" << trimmedCmd;

        // SpeakFromText(trimmedCmd);

        return true;
        response->is_ok = false;
    }

    extracted_command.fromString(trimmedCmd);

    yDebug() << "[DialogComponent::CommandManager] Number of elements in bottle: " << extracted_command.size();
    // In the first position we have the general thing to do, like: explain, next_poi, end_tour
    auto languageActionTopicList = extracted_command.get(0).asList();

    std::string newLang = languageActionTopicList->get(0).asString();
    yDebug() << "DialogComponent::CommandManager" << __LINE__;
    std::string action = languageActionTopicList->get(1).asString();
    yDebug() << "DialogComponent::CommandManager" << __LINE__;
    std::string dance = languageActionTopicList->get(2).asString();
    yDebug() << "DialogComponent::CommandManager" << __LINE__;

    response->language = newLang;
    response->dance = dance;

    // Get the poi object from the Tour manager
    PoI currentPoI;
    if (!m_tourStorage->GetTour().getPoI(m_currentPoiName, newLang, currentPoI))
    {
        yError() << "[DialogComponent::CommandManager] Unable to get the current PoI for " << m_currentPoiName;
        return false;
    }
    // Generic PoI
    PoI genericPoI;
    if (!m_tourStorage->GetTour().getPoI("___generic___", newLang, genericPoI))
    {
        yError() << "[DialogComponent::CommandManager] Unable to get the generic PoI";
        return false;
    }

    // Let's check what to do with the action
    if (action == "epl1")
    {
        // Check the fourth element to get the topic of the request
        std::string topic = languageActionTopicList->get(3).asString();

        if (topic == "function")
        {
            response->context = "explainFunction";
            response->is_ok = true;
        }
        else if (topic == "description" || topic == "descriptions" ||
                 topic == "decoration" || topic == "decorations")
        {
            response->context = "explainDescription";
            response->is_ok = true;
        }
        else
        {
            yError() << "[DialogComponent::CommandManager] Unable to assign a known topic: " << topic;
            return false;
            response->is_ok = false;
        }
    }
    else if (action == "museum")
    {

        response->is_ok = true;
        response->context = "museum";
    }
    else if (action == "general" || action == "cmd_unknown")
    {
        response->is_ok = true;
        response->context = "general";
    }
    else if (action == "next_poi" || action == "start_tour") // means that it has been found // NEXT POI
    {

        m_state = SUCCESS;
        yInfo() << "[DialogComponent::CommandManager] Next Poi Detected" << __LINE__;
        m_verbalOutputBatchReader.setDialogPhaseActive(false);
        SetFaceExpression("happy");
        

        response->is_ok = true;
        response->is_poi_ended = true;
    }
    else if (action == "end_tour") // END TOUR
    {
        yInfo() << "[DialogComponent::CommandManager] End Tour Detected" << __LINE__;
        m_verbalOutputBatchReader.setDialogPhaseActive(false);
        SetFaceExpression("happy");

        response->is_ok = true;
        response->is_poi_ended = true;
        m_state = SUCCESS;
    }
    else
    {
        yInfo() << "[DialogComponent::CommandManager] cannot interpret message " << trimmedCmd << " " << __LINE__;
        response->is_ok = false;
    }

    yDebug() << "[DialogComponent::CommandManager] End of command manager" << __LINE__;

    return true;
}

bool DialogComponent::UpdatePoILLMPrompt()
{
    
    auto requestGetCurrentPoi = std::make_shared<scheduler_interfaces::srv::GetCurrentPoi::Request>();
    while (!clientGetCurrentPoi->wait_for_service(std::chrono::milliseconds(100)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GetCurrentPoi'. Exiting.");
        }
    }
    // send the request
    auto resultGetCurrentPoi = clientGetCurrentPoi->async_send_request(requestGetCurrentPoi);
    auto futureResultGetCurrentPoi = rclcpp::spin_until_future_complete(nodeGetCurrentPoi, resultGetCurrentPoi);
    auto responseGetCurrentPoi = resultGetCurrentPoi.get();
    yDebug() << "[DialogComponent::UpdatePoILLMPrompt] responseGetCurrentPoi" << responseGetCurrentPoi->poi_name << __LINE__;

    if (futureResultGetCurrentPoi == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (responseGetCurrentPoi->is_ok == true)
        {
            m_currentPoiName = responseGetCurrentPoi->poi_name;
            // Set poi chat prompt
            if (m_currentPoiName == "madama_start")
            {
                m_iPoiChat->deleteConversation();
                m_iPoiChat->setPrompt(m_startPrompt);
            }
            else
            {
                m_iPoiChat->deleteConversation();
                m_iPoiChat->setPrompt(m_poiPrompt);
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error in getting the current poi");
            return false;
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error in getting the current poi");
        return false;
    }
    return true;
}

void DialogComponent::SetLanguage(const std::shared_ptr<dialog_interfaces::srv::SetLanguage::Request> request,
                                  std::shared_ptr<dialog_interfaces::srv::SetLanguage::Response> response)
{
    // Assume everything is going to be ok
    // If something goes wrong, we will set is_ok to false
    response->is_ok = true;

    std::string newLang = request->language;

    std::cout << "Setting language to " << newLang << std::endl;

    if (!m_tourStorage->m_loadedTour.setCurrentLanguage(newLang))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cannot set language to tour storage");
        response->is_ok = false;
        return;
    }

    yInfo() << "[DialogComponent::SetLanguage] Set Language Detected: " << newLang << __LINE__;
    // Calls the set language service of the scheduler component
    auto schedulerSetLangRequest = std::make_shared<scheduler_interfaces::srv::SetLanguage::Request>();
    schedulerSetLangRequest->language = newLang;
    while (!setLangClient->wait_for_service(std::chrono::milliseconds(100)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'Scheduler Component SetLanguage'. Exiting.");
            response->is_ok = false;
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for SchedulerComponent/SetLanguage Service");
    }
    auto result = setLangClient->async_send_request(schedulerSetLangRequest);
    if (rclcpp::spin_until_future_complete(setLangClientNode, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Language succeeded");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_language");
        response->is_ok = false;
        return;
    }

    // Calls the set language service of the text to speech component
    auto request2 = std::make_shared<text_to_speech_interfaces::srv::SetLanguage::Request>();
    request2->new_language = newLang;
    while (!setLangClient2->wait_for_service(std::chrono::milliseconds(100)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setLangClient'. Exiting.");
            response->is_ok = false;
            return;
        }
    }
    auto result2 = setLangClient2->async_send_request(request2);
    if (rclcpp::spin_until_future_complete(setLangClientNode2, result2) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Language succeeded");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_language");
        response->is_ok = false;
        return;
    }

    
    auto request3 = std::make_shared<text_to_speech_interfaces::srv::SetVoice::Request>();
    request3->new_voice = m_voicesMap[newLang];
    while (!setVoiceClient->wait_for_service(std::chrono::milliseconds(100)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setVoiceClient'. Exiting.");
            response->is_ok = false;
            return;
        }
    }
    auto result3 = setVoiceClient->async_send_request(request3);
    if (rclcpp::spin_until_future_complete(setVoiceClientNode, result3) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Voice succeeded");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_voice");
        response->is_ok = false;
        return;
    }

    
    auto request4 = std::make_shared<text_to_speech_interfaces::srv::SetLanguage::Request>();
    request4->new_language = newLang;
    while (!setSpeechToTextLanguageClient->wait_for_service(std::chrono::milliseconds(100)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setVoiceClient'. Exiting.");
            response->is_ok = false;
            return;
        }
    }
    auto result4 = setSpeechToTextLanguageClient->async_send_request(request4);
    if (rclcpp::spin_until_future_complete(setSpeechToTextLanguageClientNode, result4) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Speech to Text Language succeeded");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_speech_to_text_language");
        response->is_ok = false;
        return;
    }

}

bool DialogComponent::WaitForSpeakStart()
{
    
    auto isSpeakingRequest = std::make_shared<text_to_speech_interfaces::srv::IsSpeaking::Request>();
    while (!isSpeakingClient->wait_for_service(std::chrono::milliseconds(100)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'isSpeakingClient'. Exiting.");
        }
    }

    auto startTime = std::chrono::steady_clock::now();

    bool isSpeaking;
    do
    {
        // calls the isSpeaking service

        auto isSpeakingResult = isSpeakingClient->async_send_request(isSpeakingRequest);
        auto futureIsSpeakingResult = rclcpp::spin_until_future_complete(isSpeakingClientNode, isSpeakingResult);
        auto isSpeakingResponse = isSpeakingResult.get();
        isSpeaking = isSpeakingResponse->is_speaking;

        yInfo() << "Waiting for speak to start, IsSpeaking: " << isSpeaking << __LINE__;
        RCLCPP_INFO_STREAM(m_node->get_logger(), "Waiting for speak to start, IsSpeaking: " << isSpeaking << __LINE__);

        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
        if (elapsedTime > 10) // Timeout after 10 seconds
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Timeout while waiting for speech to start.");
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    } while (!isSpeaking);
    return true;
}

void DialogComponent::WaitForSpeakEnd()
{

    auto isSpeakingRequest = std::make_shared<text_to_speech_interfaces::srv::IsSpeaking::Request>();
    while (!isSpeakingClient->wait_for_service(std::chrono::milliseconds(100)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'isSpeakingClient'. Exiting.");
        }
    }

    bool isSpeaking;
    do
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // calls the isSpeaking service

        auto isSpeakingResult = isSpeakingClient->async_send_request(isSpeakingRequest);
        auto futureIsSpeakingResult = rclcpp::spin_until_future_complete(isSpeakingClientNode, isSpeakingResult);
        auto isSpeakingResponse = isSpeakingResult.get();
        isSpeaking = isSpeakingResponse->is_speaking;

        int secondsLeft = isSpeakingResponse->seconds_left;

        yInfo() << "IsSpeaking " << isSpeaking << " and seconds left: " << secondsLeft << __LINE__;

    } while (isSpeaking);
}

void DialogComponent::InterpretCommand(const std::shared_ptr<dialog_interfaces::srv::InterpretCommand::Request> request,
                                       std::shared_ptr<dialog_interfaces::srv::InterpretCommand::Response> response)
{
    // Assume everything is going to be ok
    // If something goes wrong, we will set is_ok to false
    response->is_ok = true;

    std::string command = request->context;

    // Get the poi object from the Tour manager
    PoI currentPoI;
    if (!m_tourStorage->GetTour().getPoI(m_currentPoiName, currentPoI))
    {
        yError() << "[DialogComponent::CommandManager] Unable to get the current PoI name: " << m_currentPoiName;
        response->is_ok = false;
        return;
    }
    // Generic PoI
    PoI genericPoI;
    if (!m_tourStorage->GetTour().getPoI("___generic___", genericPoI))
    {
        yError() << "[DialogComponent::CommandManager] Unable to get the generic PoI";
        response->is_ok = false;
        return;
    }

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

            std::string speakAction = "";

            std::vector<std::string> replies;
            std::vector<std::string> dances;

            for (const auto &action : tempActions)
            {
                std::cout << "Action Type: " << action.getType() << " with param: " << action.getParam() << " and blocking: " << action.isBlocking() << " and dance " << action.getDance() << std::endl;

                std::cout << "speak id " << ActionTypes::SPEAK << " signal id " << ActionTypes::SIGNAL << std::endl;
                switch (action.getType())
                {
                case ActionTypes::SPEAK:
                {
                    speakAction += action.getParam() + " "; // Concatenate all the speak actions
                    replies.push_back(action.getParam());
                    dances.push_back(action.getDance());

                    response->reply = replies;
                    response->dance = dances;
                    m_number_of_predefined_answers = replies.size();
                    break;
                }
                default:
                {
                    yError() << "[DialogComponent::InterpretCommand] I got an unknown ActionType.";
                    response->is_ok = false;
                    return;
                }
                }
            }
        }
    }
}

// WaitForInteraction action fragment of code start

rclcpp_action::GoalResponse DialogComponent::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const dialog_interfaces::action::WaitForInteraction::Goal> goal)
{
    RCLCPP_INFO(m_node->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DialogComponent::handle_cancel(
    const std::shared_ptr<GoalHandleWaitForInteraction> goal_handle)
{
    RCLCPP_INFO(m_node->get_logger(), "Received request to cancel goal");

    // Let's stop the current interaction and reset the state of the component

    return rclcpp_action::CancelResponse::ACCEPT;
}

void DialogComponent::handle_accepted(const std::shared_ptr<GoalHandleWaitForInteraction> goal_handle)
{
    RCLCPP_INFO(m_node->get_logger(), "Accepted goal");
    std::thread([this, goal_handle]()
                { this->WaitForInteraction(goal_handle); })
        .detach();
}

void DialogComponent::WaitForInteraction(const std::shared_ptr<GoalHandleWaitForInteraction> goal_handle)
{

    m_verbalOutputBatchReader.setDialogPhaseActive(true);

    RCLCPP_INFO(m_node->get_logger(), "Waiting for interaction");
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<dialog_interfaces::action::WaitForInteraction::Feedback>();
    feedback->status = "Waiting for interaction";
    auto result = std::make_shared<dialog_interfaces::action::WaitForInteraction::Result>();
    if (!UpdatePoILLMPrompt())
    {
        yError() << "[DialogComponent::CommandManager] Error in UpdatePoILLMPrompt";
        result->is_ok = false;
        goal_handle->abort(result);
        return;
    }

    if (goal->is_beginning_of_conversation)
    {
        m_replies.clear();
        m_last_received_interaction = "";
        yDebug() << "[DialogComponent::WaitForInteraction] Beginning of conversation detected, clearing replies" << __LINE__;
    }

    std::string questionText = "";
    float confidence = 0.0;

    // added a keyboard interaction to the goal request for debugging purposes
    // when goal->keyboard_interaction is not empty, it means that the interaction is coming from the keyboard
    if (goal->keyboard_interaction == "")
    {
        yInfo() << "[DialogComponent::WaitForInteraction] Trying to read from speechToText Port" << __LINE__;

        yarp::os::Bottle *vocalInteraction = nullptr;

        do
        {
            vocalInteraction = m_speechToTextPort.read(false); // Read from the port without blocking

            if (goal_handle->is_canceling())
            {
                result->is_ok = false;
                result->interaction = "";
                goal_handle->canceled(result);
                RCLCPP_INFO(m_node->get_logger(), "Goal canceled");
                return;
            }

            // Publish feedback
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO_THROTTLE(m_node->get_logger(), *m_node->get_clock(), 1000, "Publish feedback");

            // wait for a while before trying to read again
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

        } while (vocalInteraction == nullptr);

        if (vocalInteraction)
        {
            questionText = vocalInteraction->get(0).asString();
            confidence = vocalInteraction->get(1).asFloat32();
            yInfo() << "[DialogComponent::WaitForInteraction] Transcribed text:" << questionText << " with confidence:" << confidence;
        }
        else
        {
            yError() << "[DialogComponent::WaitForInteraction] Failed to read transcribed text";
        }

        yInfo() << "[DialogComponent::WaitForInteraction] Call received" << __LINE__;
    }
    else
    {
        yInfo() << "[DialogComponent::WaitForInteraction] call received: " << goal->keyboard_interaction << __LINE__;
        questionText = goal->keyboard_interaction;
        confidence = 1.0;
    }


    if (questionText != "") {
        SetFaceExpression("thinking");
    }

    m_last_received_interaction = questionText;
    result->is_ok = true;
    result->interaction = questionText;
    result->confidence = confidence;
    goal_handle->succeed(result);
    RCLCPP_INFO(m_node->get_logger(), "Goal succeeded");
}

// WaitForInteraction action fragment of code end

void DialogComponent::ExecuteDance(std::string danceName, float estimatedSpeechTime)
{

    // ---------------------------------Execute Dance Component Service ExecuteDance------------------------------
    yInfo() << "[DialogComponent::ExecuteDance] Starting Execute Dance Service";
    
    auto dance_request = std::make_shared<execute_dance_interfaces::srv::ExecuteDance::Request>();
    dance_request->dance_name = danceName;
    if (estimatedSpeechTime > 0.0)
    {
        dance_request->speech_time = estimatedSpeechTime;
    }
    
    // Wait for service
    while (!danceClient->wait_for_service(std::chrono::milliseconds(100)))
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

void DialogComponent::ResetDance()
{
    // ---------------------------------Execute Dance Component Service resetDance------------------------------
    yInfo() << "[DialogComponent::ResetDance] Starting Reset Dance Service";
    
    auto dance_request = std::make_shared<execute_dance_interfaces::srv::ResetDance::Request>();
    // Wait for service
    while (!danceClient->wait_for_service(std::chrono::milliseconds(100)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'ExecuteDance'. Exiting.");
        }
    }
    auto dance_result = danceClient->async_send_request(dance_request);

    if (rclcpp::spin_until_future_complete(executeDanceClientNode, dance_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reset Dance succeeded");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service reset_dance");
        return;
    }
}

// Execute Pointing action service client
void DialogComponent::ExecutePointing(std::string pointingTarget)
{

    // ---------------------------------Text to Speech Service SPEAK------------------------------
    yInfo() << "[DialogComponent::ExecutePointing] Starting Cartesian Pointing Service";
    
    auto pointing_request = std::make_shared<cartesian_pointing_interfaces::srv::PointAt::Request>();
    pointing_request->target_name = pointingTarget;
    // Wait for service
    while (!pointingClient->wait_for_service(std::chrono::milliseconds(100)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service '/CartesianPointingComponent/PointAt'. Exiting.");
        }
    }
    auto pointing_result = pointingClient->async_send_request(pointing_request);

    if (rclcpp::spin_until_future_complete(executePointingClientNode, pointing_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execute Pointing succeeded");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service point_at");
        return;
    }
}

void DialogComponent::ShortenReply(const std::shared_ptr<dialog_interfaces::srv::ShortenReply::Request> request,
                                   std::shared_ptr<dialog_interfaces::srv::ShortenReply::Response> response)
{

    yDebug() << "[DialogComponent::ShortenReply] call received with request: " << request->interaction;
    yDebug() << "[DialogComponent::ShortenReply] call received with context: " << request->context;

    std::cout << "DialogComponent::ShortenReply call received" << __LINE__ << std::endl;
    for (auto &reply : m_replies[request->context])
    {
        std::cout << "Reply at index " << &reply - &m_replies[request->context][0] << ": " << reply << std::endl;
    }
    std::cout << "Request index: " << request->duplicate_index << std::endl;

    std::string previousReply = m_replies[request->context][request->duplicate_index];

    std::string LLMQuestion = "You have just received a question: " + request->interaction + ". " +
                              "You have to answer it, but you have to take into account that the user has already received a similar answer. " +
                              "The previous answer was: " + previousReply + ". " +
                              "Please maintain the context and shorten it to a single sentence. Be careful to reply in the same language of the question!!!";


    yarp::dev::LLM_Message answer;

    if (request->context == "general" || request->context == "explainFunction" || request->context == "explainDescription")
    {
        if (!m_iGenericChat->ask(LLMQuestion, answer))
        {
            yError() << "[DialogComponent::ShortenReply] Unable to interact with chatGPT with question: " << request->interaction;
        }
    }
    else if (request->context == "museum")
    {
        if (!m_iMuseumChat->ask(LLMQuestion, answer))
        {
            yError() << "[DialogComponent::ShortenReply] Unable to interact with chatGPT with question: " << request->interaction;
        }
    }
    else
    {
        yError() << "[DialogComponent::ShortenReply] Unknown context: " << request->context;
        response->is_ok = false;
        return;
    }

    std::string answerText = answer.content;

    std::cout << "The answer is: " << answerText << std::endl;

    std::vector<std::string> answers;
    answers.push_back(answerText);

    response->reply = answers;

    response->is_ok = true;
}

void DialogComponent::Answer(const std::shared_ptr<dialog_interfaces::srv::Answer::Request> request,
                             std::shared_ptr<dialog_interfaces::srv::Answer::Response> response)
{

    std::string LLMQuestion = "You have received a question: " + request->interaction + ". " +
                              "You have to answer it while maintaining the context of the conversation. Be careful to reply in the same language of the question!!!";

    std::chrono::duration wait_ms = 200ms;
    yarp::dev::LLM_Message answer;

    if (request->context == "general")
    {
        if (!m_iGenericChat->ask(LLMQuestion, answer))
        {
            yError() << "[DialogComponent::Answer] Unable to interact with chatGPT with question: " << request->interaction;
        }
    }
    else if (request->context == "museum")
    {
        if (!m_iMuseumChat->ask(LLMQuestion, answer))
        {
            yError() << "[DialogComponent::Answer] Unable to interact with chatGPT with question: " << request->interaction;
        }
    }
    else
    {
        yError() << "[DialogComponent::Answer] Unknown context: " << request->context;
        response->is_ok = false;
        return;
    }

    std::string answerText = answer.content;

    m_replies[request->context].push_back(answerText);

    std::cout << "The answer is: " << answerText << std::endl;

    std::vector<std::string> answers;
    answers.push_back(answerText);

    response->reply = answers;
    response->is_ok = true;
}

// Speak action fragment of code start

rclcpp_action::GoalResponse DialogComponent::handle_speak_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const dialog_interfaces::action::Speak::Goal> goal)
{
    RCLCPP_INFO(m_node->get_logger(), "Received speak goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DialogComponent::handle_speak_cancel(
    const std::shared_ptr<GoalHandleSpeak> goal_handle)
{
    RCLCPP_INFO(m_node->get_logger(), "Received request to cancel speak goal");

    // Let's stop the current interaction and reset the state of the component
    // reset the verbal output queue
    m_verbalOutputBatchReader.resetQueue();
    // stop speaking
    m_audioPort.interrupt();

    return rclcpp_action::CancelResponse::ACCEPT;
}

void DialogComponent::handle_speak_accepted(const std::shared_ptr<GoalHandleSpeak> goal_handle)
{
    RCLCPP_INFO(m_node->get_logger(), "Accepted speak goal");
    std::thread([this, goal_handle]()
                { this->Speak(goal_handle); })
        .detach();
}

void DialogComponent::Speak(const std::shared_ptr<GoalHandleSpeak> goal_handle)
{   

    RCLCPP_INFO(m_node->get_logger(), "Starting Speak");
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<dialog_interfaces::action::Speak::Feedback>();
    feedback->status = "Speaking";
    auto result = std::make_shared<dialog_interfaces::action::Speak::Result>();

    std::vector<std::string> dances = goal->dances;

    std::vector<std::string> texts = goal->texts;

    if (dances.size() != texts.size())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Dances and texts vectors must have the same size");
        result->is_ok = false;
        goal_handle->abort(result);
        return;
    }

    std::unique_ptr<yarp::sig::Sound> verbalOutput = nullptr;

    auto start_time = std::chrono::high_resolution_clock::now();

    do
    {
        verbalOutput = m_verbalOutputBatchReader.GetVerbalOutput();
        if (goal_handle->is_canceling())
        {
            result->is_ok = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(m_node->get_logger(), "Goal canceled");
            return;
        }

        // Publish feedback
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO_THROTTLE(m_node->get_logger(), *m_node->get_clock(), 1000, "Waiting for verbal output");

        // wait for a while before trying to read again
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        auto elapsed_time = std::chrono::high_resolution_clock::now() - start_time;
        if (elapsed_time > std::chrono::seconds(10)) {
            RCLCPP_WARN(m_node->get_logger(), "Timeout waiting for verbal output");
            break;
        }

    } while (verbalOutput == nullptr);

    if (verbalOutput == nullptr) {
        SetFaceExpression("happy");
        m_predefined_answer_index = 0;
        m_number_of_predefined_answers = 0;
        result->is_reply_finished = true;
        RCLCPP_INFO(m_node->get_logger(), "Verbal Output Not received, get back to navigation position");
        std::string navigation_position = "navigation_position";
        ExecuteDance(navigation_position, 0); // Go back to navigation position
        m_verbalOutputBatchReader.setDialogPhaseActive(false);
        m_verbalOutputBatchReader.resetQueue();
        result->is_ok = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(m_node->get_logger(), "Goal succeeded");
        return;
    }

    yarp::sig::Sound &sound = m_audioPort.prepare();
    std::cout << "[DialogComponent::SpeakFromAudio] Preparing to speak" << std::endl;
    sound.clear();
    std::cout << "[DialogComponent::SpeakFromAudio] Cleared sound buffer" << std::endl;

    sound = *verbalOutput;
    std::cout << "[DialogComponent::SpeakFromAudio] Copied sound data" << std::endl;

    std::string dance = dances[m_predefined_answer_index];

    SetFaceExpression("happy");

    m_audioPort.write();

    std::cout << "[DialogComponent::SpeakFromAudio] Audio written to port" << std::endl;

    if (dance != "none")
    {   
        // Wait for a maximum of 10 seconds, if the return is false, we skip the dance
        if (WaitForSpeakStart()){
            if (dance.find("point") != std::string::npos)
            {
                std::cout << "[DialogComponent::SpeakFromAudio] Pointing detected, executing pointing" << std::endl;
                std::string danceTarget = dance.substr(dance.find("::") + 2);
                ExecutePointing(danceTarget);
            }
            else
            {

                std::cout << "[DialogComponent::SpeakFromAudio] Sending audio to port" << std::endl;

                float estimatedSpeechTime = sound.getDuration();

                std::cout << "[DialogComponent::SpeakFromAudio] Speak request sent with estimated speech time: " << estimatedSpeechTime << std::endl;

                yInfo() << "[DialogComponent::CommandManager] Dance detected: " << dance;
                ExecuteDance(dance, estimatedSpeechTime);
            }
        }
        else {
            yInfo() << "[DialogComponent::CommandManager] Speak did not start in time, skipping dance: " << dance;
            yInfo() << "[DialogComponent::CommandManager] May the connection between dialog component and audio player be down?";
        }
    }
    else
    {
        yInfo() << "[DialogComponent::CommandManager] No dance detected";
    }

    std::cout << "[DialogComponent::SpeakFromAudio] Waiting for speak end" << std::endl;

    std::chrono::duration wait_ms = 2000ms;
    std::this_thread::sleep_for(wait_ms);
    WaitForSpeakEnd();

    std::cout << "[DialogComponent::SpeakFromAudio] Speak ended" << std::endl;

    // Reset dance
    ResetDance();
    if (dance.find("point") != std::string::npos)
    {
        // Go back to navigation position after pointing
        WaitForPointingEnd();
    }

    m_predefined_answer_index++; // Reset the index of the predefined answer

    result->is_reply_finished = false; // If it is not the last one, we are not finished with the reply
    if (m_predefined_answer_index >= m_number_of_predefined_answers)
    {
        m_predefined_answer_index = 0;
        m_number_of_predefined_answers = 0;
        result->is_reply_finished = true;
        RCLCPP_INFO(m_node->get_logger(), "Reply finished, get back to navigation position");
        std::string navigation_position = "navigation_position";
        ExecuteDance(navigation_position, 0); // Go back to navigation position
        m_verbalOutputBatchReader.setDialogPhaseActive(false);
        m_verbalOutputBatchReader.resetQueue();
    }

    result->is_ok = true;

    goal_handle->succeed(result);
    RCLCPP_INFO(m_node->get_logger(), "Goal succeeded");
}

// Speak action fragment of code end

void DialogComponent::WaitForPointingEnd() {
    
    auto isMotionDoneRequest = std::make_shared<cartesian_pointing_interfaces::srv::IsMotionDone::Request>();
    while (!isMotionDoneClient->wait_for_service(std::chrono::milliseconds(100)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'isMotionDoneClient'. Exiting.");
        }
    }

    bool isMotionDone;
    int counter = 0;
    do
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // calls the isMotionDone service

        auto isMotionDoneResult = isMotionDoneClient->async_send_request(isMotionDoneRequest);
        auto futureIsMotionDoneResult = rclcpp::spin_until_future_complete(isMotionDoneClientNode, isMotionDoneResult);
        auto isMotionDoneResponse = isMotionDoneResult.get();
        isMotionDone = isMotionDoneResponse->is_done;

        yInfo() << "IsMotionDone " << isMotionDone << __LINE__ << counter++;

    } while (!isMotionDone);
}

void DialogComponent::SetFaceExpression(std::string expressionName) {

    yarp::os::Bottle request, reply;

    if (expressionName == "happy")
        request.fromString("emotion 1"); //happy
    else if (expressionName == "sad")
        request.fromString("emotion 0"); //sad
    else if (expressionName == "thinking")
        request.fromString("emotion 2"); //thinking
    else
        request.fromString("emotion 1"); //happy
 
    m_faceexpression_rpc_port.write(request,reply);
}
