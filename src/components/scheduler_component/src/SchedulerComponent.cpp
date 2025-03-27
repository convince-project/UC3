/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "SchedulerComponent.h"

bool SchedulerComponent::start(int argc, char*argv[])
{
    if (argc >= 2)
    {
        m_tourStorage = std::make_shared<TourStorage>(); // Loads the tour json from the file and saves a reference to the class.
        if( !m_tourStorage->LoadTour(argv[1], argv[2]))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error loading tour");
            return false;
        }
    }
    else
    {
        std::cerr << "Error: file path is missing" << std::endl;
        return false;
    }
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("SchedulerComponentNode");
    m_updatePoiService = m_node->create_service<scheduler_interfaces::srv::UpdatePoi>("/SchedulerComponent/UpdatePoi",
                                                                                std::bind(&SchedulerComponent::UpdatePoi,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_resetService = m_node->create_service<scheduler_interfaces::srv::Reset>("/SchedulerComponent/Reset",
                                                                                std::bind(&SchedulerComponent::Reset,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_endTourService = m_node->create_service<scheduler_interfaces::srv::EndTour>("/SchedulerComponent/EndTour",
                                                                                std::bind(&SchedulerComponent::EndTour,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getCurrentPoiService = m_node->create_service<scheduler_interfaces::srv::GetCurrentPoi>("/SchedulerComponent/GetCurrentPoi",
                                                                                std::bind(&SchedulerComponent::GetCurrentPoi,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getCurrentActionService = m_node->create_service<scheduler_interfaces::srv::GetCurrentAction>("/SchedulerComponent/GetCurrentAction",
                                                                                std::bind(&SchedulerComponent::GetCurrentAction,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_updateActionService = m_node->create_service<scheduler_interfaces::srv::UpdateAction>("/SchedulerComponent/UpdateAction",
                                                                                std::bind(&SchedulerComponent::UpdateAction,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getCurrentLanguageService = m_node->create_service<scheduler_interfaces::srv::GetCurrentLanguage>("/SchedulerComponent/GetCurrentLanguage",
                                                                                std::bind(&SchedulerComponent::GetCurrentLanguage,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_setLanguageService = m_node->create_service<scheduler_interfaces::srv::SetLanguage>("/SchedulerComponent/SetLanguage",
                                                                                std::bind(&SchedulerComponent::SetLanguage,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getCurrentCommandService = m_node->create_service<scheduler_interfaces::srv::GetCurrentCommand>("/SchedulerComponent/GetCurrentCommand",
                                                                                std::bind(&SchedulerComponent::GetCurrentCommand,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_setCommandService = m_node->create_service<scheduler_interfaces::srv::SetCommand>("/SchedulerComponent/SetCommand",
                                                                                std::bind(&SchedulerComponent::SetCommand,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_getAvailableCommandsService = m_node->create_service<scheduler_interfaces::srv::GetAvailableCommands>("/SchedulerComponent/GetAvailableCommands",
                                                                                std::bind(&SchedulerComponent::GetAvailableCommands,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_setPoiService = m_node->create_service<scheduler_interfaces::srv::SetPoi>("/SchedulerComponent/SetPoi",
                                                                                std::bind(&SchedulerComponent::SetPoi,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));

    RCLCPP_DEBUG(m_node->get_logger(), "SchedulerComponent::start");
    m_publisher = m_node->create_publisher<std_msgs::msg::String>("/LogComponent/add_to_log", 10);
    return true;

}

bool SchedulerComponent::close()
{
    rclcpp::shutdown();
    return true;
}

void SchedulerComponent::spin()
{
    rclcpp::spin(m_node);
}

void SchedulerComponent::publisher(std::string text)
{
    std_msgs::msg::String msg;
    msg.data = text;
    m_publisher->publish(msg);
}

void SchedulerComponent::Reset([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::Reset::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::Reset::Response>      response)
{
    RCLCPP_INFO(m_node->get_logger(), "SchedulerComponent::Reset " );
    m_currentPoi = 0;
    m_currentAction = 0;
    response->is_ok = true;
}


void SchedulerComponent::EndTour([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::EndTour::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::EndTour::Response>      response)
{
    RCLCPP_INFO(m_node->get_logger(), "SchedulerComponent::EndTour " );
    m_currentPoi = m_tourStorage->GetTour().getPoIsList().size() - 1;
    m_currentAction = 0;
    response->is_ok = true;
}

void SchedulerComponent::UpdatePoi([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::UpdatePoi::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::UpdatePoi::Response>      response)
{
    RCLCPP_INFO(m_node->get_logger(), "SchedulerComponent::UpdatePoi " );
    m_currentPoi = (m_currentPoi + 1) % m_tourStorage->GetTour().getPoIsList().size();
    m_currentAction = 0;
    response->is_ok = true;
    std::string text = "Update Poi to: " + std::to_string(m_currentPoi) + " - " + m_tourStorage->GetTour().getPoIsList()[m_currentPoi];
    publisher(text);
}

void SchedulerComponent::SetPoi([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::SetPoi::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::SetPoi::Response>      response)
{
    RCLCPP_INFO(m_node->get_logger(), "SchedulerComponent::SetPoi %d",  request->poi_number);
    int32_t old_poi_number = m_currentPoi;
    m_currentPoi = (request->poi_number) % m_tourStorage->GetTour().getPoIsList().size();
    response->is_ok = true;
    std::string text = "Update Poi to: " + std::to_string(m_currentPoi) + " - " + m_tourStorage->GetTour().getPoIsList()[m_currentPoi];
    if(old_poi_number != m_currentPoi)
    {
        publisher(text);
    }
}


void SchedulerComponent::GetCurrentPoi([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetCurrentPoi::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::GetCurrentPoi::Response>      response)
{
    response->poi_name = m_tourStorage->GetTour().getPoIsList()[m_currentPoi];
    RCLCPP_INFO(m_node->get_logger(), "SchedulerComponent::GetCurrentPoi name: %s", response->poi_name.c_str());
    response->poi_number = m_currentPoi;
    response->is_ok = true;
}

void SchedulerComponent::UpdateAction([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::UpdateAction::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::UpdateAction::Response>      response)
{
    RCLCPP_INFO(m_node->get_logger(), "SchedulerComponent::UpdateAction  " );
    std::string poi_name = m_tourStorage->GetTour().getPoIsList()[m_currentPoi];
    std::vector<Action> actions_vec;
    if(!getActionsVector(poi_name, actions_vec))
    {
        RCLCPP_ERROR(m_node->get_logger(), "Error getting actions,  poi: %s, command: %s", poi_name.c_str(), m_currentCommand.c_str());
        response->error_msg = "Error getting actions";
        response->is_ok = false;
        return;
    }

    m_currentAction = (m_currentAction + 1);
    if(m_currentAction >= static_cast<int>(actions_vec.size()))
    {
        response->done_with_poi = true;
        m_currentAction = m_currentAction % actions_vec.size();
    }
    response->is_ok = true;
}

void SchedulerComponent::GetCurrentAction([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetCurrentAction::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::GetCurrentAction::Response>      response)
{
    RCLCPP_INFO(m_node->get_logger(), "SchedulerComponent::GetCurrentAction  " );
    std::string poi_name = m_tourStorage->GetTour().getPoIsList()[m_currentPoi];
    std::vector<Action> actions_vec;
    if(!getActionsVector(poi_name, actions_vec))
    {
        RCLCPP_ERROR(m_node->get_logger(), "Error getting actions,  poi: %s command: %s", poi_name.c_str(), m_currentCommand.c_str());
        response->error_msg = "Error getting actions";
        response->is_ok = false;
        return;
    }

    RCLCPP_INFO(m_node->get_logger(), "SchedulerComponent::GetCurrentAction poi: %s act: %d of: %d", poi_name.c_str(), m_currentAction, static_cast<int>(actions_vec.size()));
    auto curact = actions_vec[m_currentAction];
    auto actionType = curact.getType();
    json j = actionType;
    std::string actionTypeStr = j.get<std::string>();
    response->is_blocking = actions_vec[m_currentAction].isBlocking();
    response->param = actions_vec[m_currentAction].getParam();
    response->type = actionTypeStr;
    response->is_ok = true;
}

void SchedulerComponent::GetCurrentLanguage([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetCurrentLanguage::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::GetCurrentLanguage::Response>      response)
{
    RCLCPP_INFO(m_node->get_logger(), "SchedulerComponent::GetCurrentLanguage  " );

    std::string language = m_tourStorage->GetTour().getCurrentLanguage();
    response->language = language;
    response->is_ok = true;
}

void SchedulerComponent::SetLanguage([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::SetLanguage::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::SetLanguage::Response>      response)
{
    RCLCPP_INFO(m_node->get_logger(), "SchedulerComponent::SetLanguage %s", request->language.c_str() );
    if(request->language.empty())
    {
        RCLCPP_ERROR(m_node->get_logger(), "Error setting language, empty language field ");
        response->is_ok = false;
        response->error_msg = "Empty language field";
        return;
    }
    if(!m_tourStorage->GetTour().setCurrentLanguage(request->language))
    {
        RCLCPP_ERROR(m_node->get_logger(), "Error setting language, language not available: %s", request->language.c_str() );
        response->is_ok = false;
        response->error_msg = "Language not available";
        return;
    }
    response->is_ok = true;
    std::string text = "Set Language to: " + request->language;
    publisher(text);
}

void SchedulerComponent::GetCurrentCommand([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetCurrentCommand::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::GetCurrentCommand::Response>      response)
{
    RCLCPP_INFO(m_node->get_logger(), "SchedulerComponent::GetCurrentCommand " );
    response->command = m_currentCommand;
    response->is_ok = true;
}

void SchedulerComponent::SetCommand([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::SetCommand::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::SetCommand::Response>      response)
{
    RCLCPP_INFO(m_node->get_logger(), "SchedulerComponent::SetCommand %s", request->command.c_str() );
    std::string poi_name = m_tourStorage->GetTour().getPoIsList()[m_currentPoi];
    if(request->command.empty())
    {
        RCLCPP_ERROR(m_node->get_logger(), "Error setting command, empty command field");
        response->is_ok = false;
        response->error_msg = "Empty command field";
        return;
    }
    if(!checkIfCommandValid(poi_name, request->command))
    {
        RCLCPP_ERROR(m_node->get_logger(), "Error setting command, command not available: %s",  request->command.c_str());
        response->is_ok = false;
        response->error_msg = "Command not available";
        return;
    }
    m_currentAction = 0;
    m_currentCommand = request->command;
    response->is_ok = true;
    std::string text = "Set Command to: " + request->command;
    publisher(text);
}

bool SchedulerComponent::checkIfCommandValid(const std::string &poiName, const std::string command)
{
    PoI currentPoi;
    if(!m_tourStorage->GetTour().getPoI(poiName,currentPoi))
    {
        std::cout << "Error getting POI" << std::endl;
        return false;
    }
    if(currentPoi.isCommandValid(command))
    {
        return true;
    }
    PoI genericPoi;
    if(!m_tourStorage->GetTour().getPoI(GENERIC_POI_NAME,genericPoi))
    {
        std::cout << "Error getting generic POI" << std::endl;
        return false;
    }
    if(genericPoi.isCommandValid(command))
    {
        return true;
    }
    std::cout << "Error getting Command" << std::endl;
    return false;

}

bool SchedulerComponent::getActionsVector(const std::string &poiName, std::vector<Action> &actions)
{
    PoI currentPoi;
    if(!m_tourStorage->GetTour().getPoI(poiName,currentPoi))
    {
        std::cout << "Error getting POI" << std::endl;
        return false;
    }
    if(currentPoi.getActions(m_currentCommand, actions))
    {
        return true;
    }
    // Add if not in PoI search in General PoI
    PoI genericPoi;
    if(!m_tourStorage->GetTour().getPoI(GENERIC_POI_NAME,genericPoi))
    {
        std::cout << "Error getting Generic POI" << std::endl;
        return false;
    }
    if(genericPoi.getActions(m_currentCommand, actions))
    {
        return true;

    }
    std::cout << "Error getting Actions" << std::endl;
    return false;
}

void SchedulerComponent::GetAvailableCommands([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetAvailableCommands::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::GetAvailableCommands::Response>      response)
{
    RCLCPP_INFO(m_node->get_logger(), "SchedulerComponent::GetAvailableCommands " );
    std::vector<std::string> commands, commands_generic;
    PoI currentPoi;
    if(!m_tourStorage->GetTour().getPoI(m_tourStorage->GetTour().getPoIsList()[m_currentPoi], currentPoi))
    {
        std::cout << "Error getting POI" << std::endl;
        response->is_ok = false;
        return;
    }
    commands = currentPoi.getAvailableCommands();
    PoI genericPoi;
    if(!m_tourStorage->GetTour().getPoI(GENERIC_POI_NAME, genericPoi))
    {
        std::cout << "Error getting generic POI" << std::endl;
        response->is_ok = false;
        return;
    }
    commands_generic = genericPoi.getAvailableCommands();
    commands.insert(commands.end(), commands_generic.begin(), commands_generic.end());
    response->commands = commands;
    response->is_ok = true;
}
