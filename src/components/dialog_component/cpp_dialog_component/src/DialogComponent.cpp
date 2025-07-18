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
    m_predefined_answer_index = 0;
    m_predefined_answer = "";

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

    // m_WaitForInteractionService = m_node->create_service<dialog_interfaces::srv::WaitForInteraction>("/DialogComponent/WaitForInteraction",
    //                                                                                                  std::bind(&DialogComponent::WaitForInteraction,
    //                                                                                                            this,
    //                                                                                                            std::placeholders::_1,
    //                                                                                                            std::placeholders::_2));

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

    m_SpeakService = m_node->create_service<dialog_interfaces::srv::Speak>("/DialogComponent/Speak",
                                                                                   std::bind(&DialogComponent::Speak,
                                                                                             this,
                                                                                             std::placeholders::_1,
                                                                                             std::placeholders::_2));

    m_WaitForInteractionAction = rclcpp_action::create_server<dialog_interfaces::action::WaitForInteraction>(
        m_node,
        "/DialogComponent/WaitForInteractionAction",
        std::bind(&DialogComponent::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DialogComponent::handle_cancel, this, std::placeholders::_1),
        std::bind(&DialogComponent::handle_accepted, this, std::placeholders::_1));

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

    yInfo() << "DialogComponent::ManageContext ChatBot interrogation" << __LINE__;
    // Pass the question to chatGPT
    yarp::dev::LLM_Message answer;
    /// TODO: This is an awful solution. Remove it as soon as possible
    // END

    std::chrono::duration wait_ms = 200ms;
    if (!m_iPoiChat->ask(m_last_received_interaction, answer))
    {
        yError() << "[DialogComponent::ManageContext] Unable to interact with chatGPT with question: " << m_last_received_interaction;
        std::this_thread::sleep_for(wait_ms);
    }
    std::string answerText = answer.content;
    yInfo() << "[DialogComponent::ManageContext] ChatBot Output: " << answerText << __LINE__;
    // Pass the chatGPT answer to the JSON TourManager

    if (!CommandManager(answerText, response))
    {
        yError() << "[DialogComponent::ManageContext] Error in Command Manager for the command: " << answerText;
        std::this_thread::sleep_for(wait_ms);
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

        // Now let's Find that trimmedCmd in the JSON
        // if (!InterpretCommand(action, currentPoI, genericPoI))
        // {
        //     yError() << "[DialogComponent::CommandManager] Interpret action: " << action;
        //     return false;
        // }
        // return true;
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
        response->is_ok = true;
        response->is_poi_ended = true;

        // TODO Call a service maybe?
    }
    else if (action == "end_tour") // END TOUR
    {
        yInfo() << "[DialogComponent::CommandManager] End Tour Detected" << __LINE__;
        // calls the end tour service of the scheduler component

        response->is_ok = true;
        response->is_poi_ended = true;
        m_state = SUCCESS;

        // TODO call service
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
    std::shared_ptr<rclcpp::Node> nodeGetCurrentPoi = rclcpp::Node::make_shared("DialogComponentNodeGetCurrentPoi");
    std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::GetCurrentPoi>> clientGetCurrentPoi = nodeGetCurrentPoi->create_client<scheduler_interfaces::srv::GetCurrentPoi>("/SchedulerComponent/GetCurrentPoi");
    auto requestGetCurrentPoi = std::make_shared<scheduler_interfaces::srv::GetCurrentPoi::Request>();
    while (!clientGetCurrentPoi->wait_for_service(std::chrono::seconds(1)))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

    if (!m_tourStorage->m_loadedTour.setCurrentLanguage(newLang))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cannot set language to tour storage");
        response->is_ok = false;
        return;
    }

    yInfo() << "[DialogComponent::SetLanguage] Set Language Detected: " << newLang << __LINE__;
    // Calls the set language service of the scheduler component
    auto setLangClientNode = rclcpp::Node::make_shared("DialogComponentSetLangNode");
    auto setLangClient = setLangClientNode->create_client<scheduler_interfaces::srv::SetLanguage>("/SchedulerComponent/SetLanguage");
    auto schedulerSetLangRequest = std::make_shared<scheduler_interfaces::srv::SetLanguage::Request>();
    schedulerSetLangRequest->language = newLang;
    while (!setLangClient->wait_for_service(std::chrono::seconds(1)))
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
    auto setLangClientNode2 = rclcpp::Node::make_shared("DialogComponentSetLangNode2");
    auto setLangClient2 = setLangClientNode2->create_client<text_to_speech_interfaces::srv::SetLanguage>("/TextToSpeechComponent/SetLanguage");
    auto request2 = std::make_shared<text_to_speech_interfaces::srv::SetLanguage::Request>();
    request2->new_language = newLang;
    while (!setLangClient2->wait_for_service(std::chrono::seconds(1)))
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

    auto setVoiceClientNode = rclcpp::Node::make_shared("DialogComponentSetVoiceNode");
    auto setVoiceClient = setVoiceClientNode->create_client<text_to_speech_interfaces::srv::SetVoice>("/TextToSpeechComponent/SetVoice");
    auto request3 = std::make_shared<text_to_speech_interfaces::srv::SetVoice::Request>();
    request3->new_voice = m_voicesMap[newLang];
    while (!setVoiceClient->wait_for_service(std::chrono::seconds(1)))
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

            // bool containsSpeak = false;
            // float danceTime = 0.0f;

            std::string speakAction = "";

            auto action = tempActions[m_predefined_answer_index]; // Get the action at the current index

            switch (action.getType())
            {
                case ActionTypes::SPEAK:
                {
                    speakAction += action.getParam() + " "; // Concatenate all the speak actions

                    response->is_ok = true; // Set the response to ok, because we are going to speak
                    response->reply = action.getParam();

                    // check if the action is the last one in the list
                    if (m_predefined_answer_index < (tempActions.size() - 1))
                    {
                        response->is_reply_finished = false; // If it is not the last one, we are not finished with the reply
                        m_predefined_answer_index += 1;      // Increment the index of the predefined answer
                        m_predefined_answer = m_predefined_answer + " " + speakAction; // Concatenate the predefined answer with the new speak action
                    }
                    else
                    {
                        response->is_reply_finished = true; // If it is the last one, we are finished with the reply
                        m_predefined_answer_index = 0;      // Reset the index of the predefined answer
                        m_replies[command].push_back(m_predefined_answer); // Store the speak action in the replies map
                        m_predefined_answer = ""; // Reset the predefined answer
                    }

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
            RCLCPP_INFO(m_node->get_logger(), "Publish feedback");

            // wait for a while before trying to read again
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

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

    // ---------------------------------Text to Speech Service SPEAK------------------------------
    yInfo() << "[DialogComponent::ExecuteDance] Starting Execute Dance Service";
    auto executeDanceClientNode = rclcpp::Node::make_shared("ExecuteDanceComponentExecuteDanceNode");

    auto danceClient = executeDanceClientNode->create_client<execute_dance_interfaces::srv::ExecuteDance>("/ExecuteDanceComponent/ExecuteDance");
    auto dance_request = std::make_shared<execute_dance_interfaces::srv::ExecuteDance::Request>();
    dance_request->dance_name = danceName;
    dance_request->speech_time = estimatedSpeechTime;
    // Wait for service
    while (!danceClient->wait_for_service(std::chrono::seconds(1)))
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

// Protected function to actually speak given the text
void DialogComponent::ExecuteDance(std::string danceName)
{

    // ---------------------------------Text to Speech Service SPEAK------------------------------
    yInfo() << "[DialogComponent::ExecuteDance] Starting Text to Speech Service";
    auto executeDanceClientNode = rclcpp::Node::make_shared("ExecuteDanceComponentExecuteDanceNode");

    auto danceClient = executeDanceClientNode->create_client<execute_dance_interfaces::srv::ExecuteDance>("/ExecuteDanceComponent/ExecuteDance");
    auto dance_request = std::make_shared<execute_dance_interfaces::srv::ExecuteDance::Request>();
    dance_request->dance_name = danceName;
    // Wait for service
    while (!danceClient->wait_for_service(std::chrono::seconds(1)))
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
        
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service speak");
        return;
    }

    auto futureSpeakResult = rclcpp::spin_until_future_complete(setCommandClientNode, speak_result);
    if (futureSpeakResult != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service speak");
        return;
    }
    auto speakResponse = speak_result.get();

    float estimatedSpeechTime = speakResponse->speech_time;

    std::cout << "[DialogComponent::SpeakFromText] Speak request sent with estimated speech time: " << estimatedSpeechTime << std::endl;

    if (dance != "none")
    {
        yInfo() << "[DialogComponent::CommandManager] Dance detected: " << dance;
        ExecuteDance(dance, estimatedSpeechTime);
    }
    else
    {
        yInfo() << "[DialogComponent::CommandManager] No dance detected";
    }

    WaitForSpeakEnd();
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

    std::chrono::duration wait_ms = 200ms;
    yarp::dev::LLM_Message answer;

    if (request->context == "general" || request->context == "explainFunction" || request->context == "explainDescription")
    {
        if (!m_iGenericChat->ask(LLMQuestion, answer))
        {
            yError() << "[DialogComponent::ShortenReply] Unable to interact with chatGPT with question: " << request->interaction;
            std::this_thread::sleep_for(wait_ms);
        }
    }
    else if (request->context == "museum")
    {
        if (!m_iMuseumChat->ask(LLMQuestion, answer))
        {
            yError() << "[DialogComponent::ShortenReply] Unable to interact with chatGPT with question: " << request->interaction;
            std::this_thread::sleep_for(wait_ms);
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

    response->reply = answerText;

    // SpeakFromText(answerText);

    response->is_ok = true;
    response->is_reply_finished = true; // We are finished with the reply
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
            std::this_thread::sleep_for(wait_ms);
        }
    }
    else if (request->context == "museum")
    {
        if (!m_iMuseumChat->ask(LLMQuestion, answer))
        {
            yError() << "[DialogComponent::Answer] Unable to interact with chatGPT with question: " << request->interaction;
            std::this_thread::sleep_for(wait_ms);
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

    response->reply = answerText;
    response->is_ok = true;
    response->is_reply_finished = true; // We are finished with the reply
}

void DialogComponent::Speak(const std::shared_ptr<dialog_interfaces::srv::Speak::Request> request,
                            std::shared_ptr<dialog_interfaces::srv::Speak::Response> response)
{
    std::string dance = request->dance;

    std::string text = request->text;
    
    // if (dance != "none")
    // {
    //     yInfo() << "[DialogComponent::CommandManager] Dance detected: " << dance;
    //     ExecuteDance(dance, speech_time);
    //     response->is_ok = true;
    // }
    // else
    // {
    //     yInfo() << "[DialogComponent::CommandManager] No dance detected";
    // }

    SpeakFromText(text, dance);

    response->is_ok = true;
}