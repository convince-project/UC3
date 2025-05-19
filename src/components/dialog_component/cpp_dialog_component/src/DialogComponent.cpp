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
    m_jsonPath = "/home/user1/UC3/conf/tours.json";
    m_tourName = "TOUR_MADAMA_3";
    m_speechToTextClientName = "/DialogComponent/SpeechToTextClient:i";
    m_speechToTextServerName = "/SpeechToTextComponent/text:o";
    m_tourLoadedAtStart = false;
    m_currentPoiName = "sala_delle_guardie";
    // m_exit = false
    m_fallback_repeat_counter = 0;
    m_fallback_threshold = 3;
    m_state = IDLE;

    // m_duplicateIndex = -1;
    m_last_received_interaction = "";
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
        remote = "/poi_madama_chat/LLM_nws/rpc:i";

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

    yInfo() << "[DialogComponent::ConfigureYARP] Successfully configured component";
    return true;
}

bool DialogComponent::start(int argc, char *argv[])
{
    if (!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("DialogComponentNode");


    m_manageContextService = m_node->create_service<dialog_interfaces::srv::ManageContext>("/DialogComponent/ManageContext",
                                                                                           std::bind(&DialogComponent::ManageContext,
                                                                                                     this,
                                                                                                     std::placeholders::_1,
                                                                                                     std::placeholders::_2));

    m_WaitForInteractionService = m_node->create_service<dialog_interfaces::srv::WaitForInteraction>("/DialogComponent/WaitForInteraction",
                                                                                                     std::bind(&DialogComponent::WaitForInteraction,
                                                                                                               this,
                                                                                                               std::placeholders::_1,
                                                                                                               std::placeholders::_2));

    m_ShortenAndSpeakService = m_node->create_service<dialog_interfaces::srv::ShortenAndSpeak>("/DialogComponent/ShortenAndSpeak",
                                                                                               std::bind(&DialogComponent::ShortenAndSpeak,
                                                                                                         this,
                                                                                                         std::placeholders::_1,
                                                                                                         std::placeholders::_2));

    m_AnswerAndSpeakService = m_node->create_service<dialog_interfaces::srv::AnswerAndSpeak>("/DialogComponent/AnswerAndSpeak",
                                                                                             std::bind(&DialogComponent::AnswerAndSpeak,
                                                                                                       this,
                                                                                                       std::placeholders::_1,
                                                                                                       std::placeholders::_2));
    
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

void DialogComponent::ManageContext(const std::shared_ptr<dialog_interfaces::srv::ManageContext::Request> request,
                                    std::shared_ptr<dialog_interfaces::srv::ManageContext::Response> response)
{

    yInfo() << "DialogComponent::DialogExecution ChatBot interrogation" << __LINE__;
    // Pass the question to chatGPT
    yarp::dev::LLM_Message answer;
    /// TODO: This is an awful solution. Remove it as soon as possible
    // END

    std::chrono::duration wait_ms = 200ms;
    if (!m_iPoiChat->ask(m_last_received_interaction, answer))
    {
        yError() << "[DialogComponent::DialogExecution] Unable to interact with chatGPT with question: " << m_last_received_interaction;
        std::this_thread::sleep_for(wait_ms);
    }
    std::string answerText = answer.content;
    yInfo() << "DialogComponent::DialogExecution ChatBot Output: " << answerText << __LINE__;
    // Pass the chatGPT answer to the JSON TourManager

    if (!CommandManager(answerText, response))
    {
        yError() << "[DialogComponent::DialogExecution] Error in Command Manager for the command: " << answerText;
        std::this_thread::sleep_for(wait_ms);
    }

    yDebug() << "[DialogComponent::DialogExecution] Response from Command Manager: " << response->is_ok << response->needs_more_processing << response->language << response->llm_context;

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
        response->needs_more_processing = false;
    }

    extracted_command.fromString(trimmedCmd);

    yDebug() << "[DialogComponent::CommandManager] Number of elements in bottle: " << extracted_command.size();
    // In the first position we have the general thing to do, like: explain, next_poi, end_tour
    auto languageActionTopicList = extracted_command.get(0).asList();

    std::string newLang = languageActionTopicList->get(0).asString();
    yDebug() << "DialogComponent::CommandManager" << __LINE__;
    std::string action = languageActionTopicList->get(1).asString();
    yDebug() << "DialogComponent::CommandManager" << __LINE__;

    if (!SetLanguage(newLang))
    {
        yError() << "[DialogComponent::CommandManager] Unable to set the language: " << newLang;
        return false;
    }

    response->language = newLang;

    // Get the poi object from the Tour manager
    PoI currentPoI;
    if (!m_tourStorage->GetTour().getPoI(m_currentPoiName, currentPoI))
    {
        yError() << "[DialogComponent::CommandManager] Unable to get the current PoI name: " << m_currentPoiName;
        return false;
    }
    // Generic PoI
    PoI genericPoI;
    if (!m_tourStorage->GetTour().getPoI("___generic___", genericPoI))
    {
        yError() << "[DialogComponent::CommandManager] Unable to get the generic PoI";
        return false;
    }

    // Let's check what to do with the action
    if (action == "epl1")
    {
        // Check the second element for determining the exact action: artist, art_piece, historical_period, technique
        std::string topic = languageActionTopicList->get(2).asString();

        if (topic == "function")
        {
            action = "explainFunction";
            response->is_ok = true;
            response->needs_more_processing = false;
        }
        else if (topic == "description" || topic == "descriptions" ||
                 topic == "decoration" || topic == "decorations")
        {
            action = "explainDescription";
            response->is_ok = true;
            response->needs_more_processing = false;
        }
        else
        {
            yError() << "[DialogComponent::CommandManager] Unable to assign a known topic: " << topic;
            return false;
            response->is_ok = false;
            response->needs_more_processing = false;
        }

        // Horrible solution -------------------------------------------------------------- END//

        // Now let's Find that trimmedCmd in the JSON
        if (!InterpretCommand(action, currentPoI, genericPoI))
        {
            yError() << "[DialogComponent::CommandManager] Interpret action: " << action;
            return false;
        }
        return true;
    }
    else if (action == "museum")
    {

        response->needs_more_processing = true;
        response->is_ok = true;
        response->llm_context = "museum";
    }
    else if (action == "general" || action == "cmd_unknown")
    {

        response->needs_more_processing = true;
        response->is_ok = true;
        response->llm_context = "general";
    }
    else if (action == "say")
    {
        yWarning() << "The trimmedCmd:" << trimmedCmd;

        response->needs_more_processing = false;
        response->is_ok = true;

        std::chrono::duration wait_ms = 200ms;

        std::string answerText = languageActionTopicList->get(2).asString();
        // ---------------------------------Text to Speech Service SPEAK------------------------------

        // SpeakFromText(answerText);
    }
    else if (action == "next_poi" || action == "start_tour") // means that it has been found // NEXT POI
    {

        auto updatePoiClientNode = rclcpp::Node::make_shared("DialogComponentUpdatePoiNode");
        auto updatePoiClient = updatePoiClientNode->create_client<scheduler_interfaces::srv::UpdatePoi>("/SchedulerComponent/UpdatePoi");
        auto updatePoiRequest = std::make_shared<scheduler_interfaces::srv::UpdatePoi::Request>();
        while (!updatePoiClient->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'updatePoiClient'. Exiting.");
            }
        }
        auto updatePoiResult = updatePoiClient->async_send_request(updatePoiRequest);
        auto futureUpdatePoiResult = rclcpp::spin_until_future_complete(updatePoiClientNode, updatePoiResult);
        if (futureUpdatePoiResult == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (updatePoiResult.get()->is_ok)
            {
                yInfo() << "[DialogComponent::CommandManager] Update Poi Succeeded" << __LINE__;
            }
            else
            {
                yError() << "[DialogComponent::CommandManager] Update Poi Failed" << __LINE__;
            }
        }
        else
        {
            yError() << "[DialogComponent::CommandManager] Update Poi Failed" << __LINE__;
        }

        if (!UpdatePoILLMPrompt())
        {
            yError() << "[DialogComponent::CommandManager] Error in UpdatePoILLMPrompt";
            response->is_ok = false;
            return false;
        }

        m_state = SUCCESS;
        yInfo() << "[DialogComponent::CommandManager] Next Poi Detected" << __LINE__;
        response->needs_more_processing = false;
        response->is_ok = true;
        response->is_poi_ended = true;

        // TODO Call a service maybe?
    }
    else if (action == "end_tour") // END TOUR
    {
        yInfo() << "[DialogComponent::CommandManager] End Tour Detected" << __LINE__;
        // calls the end tour service of the scheduler component
        auto endTourClientNode = rclcpp::Node::make_shared("DialogComponentEndTourNode");
        auto endTourClient = endTourClientNode->create_client<scheduler_interfaces::srv::EndTour>("/SchedulerComponent/EndTour");
        auto endTourRequest = std::make_shared<scheduler_interfaces::srv::EndTour::Request>();
        while (!endTourClient->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'endTourClient'. Exiting.");
            }
        }
        auto endTourResult = endTourClient->async_send_request(endTourRequest);
        auto futureEndTourResult = rclcpp::spin_until_future_complete(endTourClientNode, endTourResult);
        if (futureEndTourResult == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (endTourResult.get()->is_ok)
            {
                yInfo() << "[DialogComponent::CommandManager] End Tour Succeeded" << __LINE__;
            }
            else
            {
                yError() << "[DialogComponent::CommandManager] End Tour Failed" << __LINE__;
            }
        }
        else
        {
            yError() << "[DialogComponent::CommandManager] End Tour Failed" << __LINE__;
        }
        response->needs_more_processing = false;
        response->is_ok = true;
        response->is_poi_ended = true;
        m_state = SUCCESS;

        // TODO call service
    }
    else
    {
        yInfo() << "[DialogComponent::CommandManager] cannot interpret message " << trimmedCmd << " " << __LINE__;
        response->needs_more_processing = false;
        response->is_ok = false;
    }

    yDebug() << "[DialogComponent::CommandManager] End of command manager" << __LINE__;

    return true;
}

bool DialogComponent::UpdatePoILLMPrompt()
{
    std::shared_ptr<rclcpp::Node> nodeGetCurrentPoi = rclcpp::Node::make_shared("DialogComponentNodeGetCurrentPoi");
    std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::GetCurrentPoi>> clientGetCurrentPoi = nodeGetCurrentPoi->create_client<scheduler_interfaces::srv::GetCurrentPoi>("/SchedulerComponent/GetCurrentPoi");
    auto requestGetCurrentPoi = std::make_shared<scheduler_interfaces::srv::GetCurrentPoi::Request>();
    while (!clientGetCurrentPoi->wait_for_service(std::chrono::seconds(1)))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'GetCurrentPoi'. Exiting.");
            return false;
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

bool DialogComponent::SetLanguage(const std::string &newLang)
{
    if (!m_tourStorage->m_loadedTour.setCurrentLanguage(newLang))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cannot set language to tour storage");
        return false;
    }

    yInfo() << "[DialogComponent::SetLanguage] Set Language Detected: " << newLang << __LINE__;
    // Calls the set language service of the scheduler component
    auto setLangClientNode = rclcpp::Node::make_shared("DialogComponentSetLangNode");
    auto setLangClient = setLangClientNode->create_client<scheduler_interfaces::srv::SetLanguage>("/SchedulerComponent/SetLanguage");
    auto request = std::make_shared<scheduler_interfaces::srv::SetLanguage::Request>();
    request->language = newLang;
    while (!setLangClient->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setLangClient'. Exiting.");
            return false;
        }
    }
    auto result = setLangClient->async_send_request(request);
    if (rclcpp::spin_until_future_complete(setLangClientNode, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Language succeeded");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_language");
        return false;
    }

    // Calls the set language service of the text to speech component
    auto setLangClientNode2 = rclcpp::Node::make_shared("DialogComponentSetLangNode2");
    auto setLangClient2 = setLangClientNode2->create_client<text_to_speech_interfaces::srv::SetLanguage>("/TextToSpeechComponent/SetLanguage");
    auto request2 = std::make_shared<text_to_speech_interfaces::srv::SetLanguage::Request>();
    request2->new_language = newLang;
    while (!setLangClient2->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setLangClient'. Exiting.");
            return false;
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
        return false;
    }

    auto setVoiceClientNode = rclcpp::Node::make_shared("DialogComponentSetVoiceNode");
    auto setVoiceClient = setVoiceClientNode->create_client<text_to_speech_interfaces::srv::SetVoice>("/TextToSpeechComponent/SetVoice");
    auto request3 = std::make_shared<text_to_speech_interfaces::srv::SetVoice::Request>();
    request3->new_voice = m_voicesMap[newLang];
    while (!setVoiceClient->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setVoiceClient'. Exiting.");
            return false;
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
        return false;
    }
    // Setting language in the speech transcriber
    yDebug() << "[DialogComponent::SetLanguage] Setting language in the speech transcriber: " << newLang << __LINE__;

    // BEGIN OF THE CODE THAT MAY BE USEFUL IN THE FUTURE

    // We cannot set the language of the speech to text component from here, because for now the speech to text component needs the language
    // to be already set in order to transcribe the speech, the code below may be useful if the yarp speech transcriber will be able
    // to transcribe the speech in any language and then set the language

    // Call speechToText component service to set the language for the yarp speech transcriber

    // auto setSpeechToTextLanguageClientNode = rclcpp::Node::make_shared("DialogComponentSetSpeechToTextLanguageNode");
    // auto setSpeechToTextLanguageClient = setSpeechToTextLanguageClientNode->create_client<text_to_speech_interfaces::srv::SetLanguage>("/SpeechToTextComponent/SetLanguage");
    // auto request4 = std::make_shared<text_to_speech_interfaces::srv::SetLanguage::Request>();
    // request4->new_language = newLang;
    // while (!setSpeechToTextLanguageClient->wait_for_service(std::chrono::seconds(1)))
    // {
    //     if (!rclcpp::ok())
    //     {
    //         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setSpeechToTextLanguageClient'. Exiting.");
    //         return false;
    //     }
    // }
    // auto result3 = setSpeechToTextLanguageClient->async_send_request(request3);
    // if (rclcpp::spin_until_future_complete(setSpeechToTextLanguageClientNode, result3) == rclcpp::FutureReturnCode::SUCCESS)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Language for Speech-to-Text component succeeded");
    // }
    // else
    // {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_language for Speech-to-Text component");
    //     return false;
    // }
    // // Setting language in the speech transcriber
    // yDebug() << "[DialogComponent::CommandManager] Setting language in the speech to text: " << newLang << __LINE__;

    // END OF THE CODE THAT MAY BE USEFUL IN THE FUTURE

    return true;
}

void DialogComponent::WaitForSpeakEnd()
{
    bool isSpeaking = false;
    do
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // calls the isSpeaking service
        auto isSpeakingClientNode = rclcpp::Node::make_shared("DialogComponentIsSpeakingNode");
        auto isSpeakingClient = isSpeakingClientNode->create_client<text_to_speech_interfaces::srv::IsSpeaking>("/TextToSpeechComponent/IsSpeaking");
        auto isSpeakingRequest = std::make_shared<text_to_speech_interfaces::srv::IsSpeaking::Request>();
        while (!isSpeakingClient->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
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

bool DialogComponent::InterpretCommand(const std::string &command, PoI currentPoI, PoI genericPoI)
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

            // bool containsSpeak = false;
            // float danceTime = 0.0f;

            std::string speakAction = "";

            for (Action action : tempActions) // Loops through all the actions until the blocking one. Execute all of them
            {
                switch (action.getType())
                {
                case ActionTypes::SPEAK:
                {

                    // Synthesize the text
                    SpeakFromText(action.getParam());

                    WaitForSpeakEnd();

                    break;
                }
                case ActionTypes::DANCE:

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

void DialogComponent::WaitForInteraction(const std::shared_ptr<dialog_interfaces::srv::WaitForInteraction::Request> request,
                                         std::shared_ptr<dialog_interfaces::srv::WaitForInteraction::Response> response)
{

    if (request->is_beginning_of_conversation)
    {
        m_replies.clear();
        m_last_received_interaction = "";
        yDebug() << "[DialogComponent::WaitForInteraction] Beginning of conversation detected, clearing replies" << __LINE__;
    }

    std::string questionText = "";

    if (request->keyboard_interaction == "")
    {
        yInfo() << "[DialogComponent::WaitForInteraction] Trying to read from speechToText Port" << __LINE__;

        yarp::os::Bottle *result = m_speechToTextPort.read();
        if (result)
        {   
            questionText = result->toString();
            yInfo() << "[DialogComponent::WaitForInteraction] Transcribed text:" << questionText << __LINE__;
        }
        else
        {
            yError() << "[DialogComponent::WaitForInteraction] Failed to read transcribed text";
        }

        yInfo() << "[DialogComponent::WaitForInteraction] Call received" << __LINE__;
    }
    else
    {
        yInfo() << "[DialogComponent::WaitForInteraction] call received: " << request->keyboard_interaction << __LINE__;
        questionText = request->keyboard_interaction;
    }

    m_last_received_interaction = questionText;
    response->is_ok = true;
    response->interaction = questionText;
}

// Protected function to actually speak given the text
void DialogComponent::SpeakFromText(std::string text)
{

    // ---------------------------------Text to Speech Service SPEAK------------------------------
    yInfo() << "[DialogComponent::SpeakFromText] Starting Text to Speech Service";
    auto setCommandClientNode = rclcpp::Node::make_shared("TextToSpeechComponentSetCommandNode");

    auto speakClient = setCommandClientNode->create_client<text_to_speech_interfaces::srv::Speak>("/TextToSpeechComponent/Speak");
    auto speak_request = std::make_shared<text_to_speech_interfaces::srv::Speak::Request>();
    speak_request->text = text;
    // Wait for service
    while (!speakClient->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setCommandClient'. Exiting.");
        }
    }
    auto speak_result = speakClient->async_send_request(speak_request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(setCommandClientNode, speak_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speak succeeded");
        WaitForSpeakEnd();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service speak");
        return;
    }
}

void DialogComponent::ShortenAndSpeak(const std::shared_ptr<dialog_interfaces::srv::ShortenAndSpeak::Request> request,
                                      std::shared_ptr<dialog_interfaces::srv::ShortenAndSpeak::Response> response)
{

    yDebug() << "[DialogComponent::ShortenAndSpeak] call received with request: " << request->interaction;
    yDebug() << "[DialogComponent::ShortenAndSpeak] call received with context: " << request->llm_context;

    std::cout << "DialogComponent::ShortenAndSpeak call received" << __LINE__ << std::endl;
    for (auto &reply : m_replies[request->llm_context])
    {
        std::cout << "Reply at index " << &reply - &m_replies[request->llm_context][0] << ": " << reply << std::endl;
    }
    std::cout << "Request index: " << request->duplicate_index << std::endl;

    std::string previousReply = m_replies[request->llm_context][request->duplicate_index];

    std::string LLMQuestion = "You have just received a question: " + request->interaction + ". " +
                              "You have to answer it, but you have to take into account that the user has already received a similar answer. " +
                              "The previous answer was: " + previousReply + ". " +
                              "Please maintain the context and shorten it to a single sentence.";

    std::chrono::duration wait_ms = 200ms;
    yarp::dev::LLM_Message answer;

    if (request->llm_context == "general")
    {
        if (!m_iGenericChat->ask(LLMQuestion, answer))
        {
            yError() << "[DialogComponent::AnswerAndSpeak] Unable to interact with chatGPT with question: " << request->interaction;
            std::this_thread::sleep_for(wait_ms);
        }
    }
    else if (request->llm_context == "museum")
    {
        if (!m_iMuseumChat->ask(LLMQuestion, answer))
        {
            yError() << "[DialogComponent::AnswerAndSpeak] Unable to interact with chatGPT with question: " << request->interaction;
            std::this_thread::sleep_for(wait_ms);
        }
    }
    else
    {
        yError() << "[DialogComponent::AnswerAndSpeak] Unknown context: " << request->llm_context;
        response->is_ok = false;
        return;
    }

    std::string answerText = answer.content;

    std::cout << "The answer is: " << answerText << std::endl;

    SpeakFromText(answerText);

    response->is_ok = true;
}

void DialogComponent::AnswerAndSpeak(const std::shared_ptr<dialog_interfaces::srv::AnswerAndSpeak::Request> request,
                                     std::shared_ptr<dialog_interfaces::srv::AnswerAndSpeak::Response> response)
{

    std::string LLMQuestion = "You have received a question: " + request->interaction + ". " +
                              "You have to answer it while maintaining the context of the conversation.";

    std::chrono::duration wait_ms = 200ms;
    yarp::dev::LLM_Message answer;

    if (request->llm_context == "general")
    {
        if (!m_iGenericChat->ask(LLMQuestion, answer))
        {
            yError() << "[DialogComponent::AnswerAndSpeak] Unable to interact with chatGPT with question: " << request->interaction;
            std::this_thread::sleep_for(wait_ms);
        }
    }
    else if (request->llm_context == "museum")
    {
        if (!m_iMuseumChat->ask(LLMQuestion, answer))
        {
            yError() << "[DialogComponent::AnswerAndSpeak] Unable to interact with chatGPT with question: " << request->interaction;
            std::this_thread::sleep_for(wait_ms);
        }
    }
    else
    {
        yError() << "[DialogComponent::AnswerAndSpeak] Unknown context: " << request->llm_context;
        response->is_ok = false;
        return;
    }

    std::string answerText = answer.content;

    m_replies[request->llm_context].push_back(answerText);

    std::cout << "The answer is: " << answerText << std::endl;

    SpeakFromText(answerText);

    response->is_ok = true;
}