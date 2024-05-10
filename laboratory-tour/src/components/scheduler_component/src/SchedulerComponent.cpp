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
            return false;
        }
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

    RCLCPP_DEBUG(m_node->get_logger(), "SchedulerComponent::start");
    std::cout << "SchedulerComponent::start";
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

void SchedulerComponent::Reset([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::Reset::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::Reset::Response>      response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    m_currentPoi = 0;
    m_currentAction = 0;
    response->is_ok = true;
}


void SchedulerComponent::EndTour([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::EndTour::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::EndTour::Response>      response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    m_currentPoi = m_tourStorage->GetTour().getPoIsList().size() - 1;
    m_currentAction = 0;
    response->is_ok = true;
}

void SchedulerComponent::UpdatePoi([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::UpdatePoi::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::UpdatePoi::Response>      response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    m_currentPoi = (m_currentPoi + 1) % m_tourStorage->GetTour().getPoIsList().size();
    m_currentAction = 0;
    response->is_ok = true;
}


void SchedulerComponent::GetCurrentPoi([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetCurrentPoi::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::GetCurrentPoi::Response>      response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    response->poi_name = m_tourStorage->GetTour().getPoIsList()[m_currentPoi];
    response->poi_number = m_currentPoi;
    response->is_ok = true;
}

void SchedulerComponent::UpdateAction([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::UpdateAction::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::UpdateAction::Response>      response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    std::string poi_name = m_tourStorage->GetTour().getPoIsList()[m_currentPoi];
    std::vector<Action> actions_vec;
    if(!getActionsVector(poi_name, actions_vec))
    {
        response->error_msg = "Error getting actions";
        response->is_ok = false;
        return;
    }

    m_currentAction = (m_currentAction + 1);
    if(m_currentAction >= actions_vec.size())
    {
        response->done_with_poi = true;
        m_currentAction = m_currentAction % actions_vec.size();
    }
    response->is_ok = true;
}

void SchedulerComponent::GetCurrentAction([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetCurrentAction::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::GetCurrentAction::Response>      response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__ );
    std::string poi_name = m_tourStorage->GetTour().getPoIsList()[m_currentPoi];
    std::vector<Action> actions_vec;
    if(!getActionsVector(poi_name, actions_vec))
    {
        response->error_msg = "Error getting actions";
        response->is_ok = false;
        return;
    }

    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__ << " " << poi_name << " " << m_currentAction << " " << actions_vec.size());
    auto curact = actions_vec[m_currentAction];
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    auto actionType = curact.getType();
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    json j = actionType;
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    std::string actionTypeStr = j.get<std::string>();

    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    response->is_blocking = actions_vec[m_currentAction].isBlocking();
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    response->param = actions_vec[m_currentAction].getParam();
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    response->type = actionTypeStr;
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    response->is_ok = true;
}

void SchedulerComponent::GetCurrentLanguage([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetCurrentLanguage::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::GetCurrentLanguage::Response>      response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);

    std::string language = m_tourStorage->GetTour().getCurrentLanguage();
    response->language = language;
    response->is_ok = true;
}

void SchedulerComponent::SetLanguage([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::SetLanguage::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::SetLanguage::Response>      response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    if(request->language.empty())
    {
        response->is_ok = false;
        response->error_msg = "Empty language field";
        return;
    }
    if(!m_tourStorage->GetTour().setCurrentLanguage(request->language))
    {
        response->is_ok = false;
        response->error_msg = "Language not available";
        return;
    }
    response->is_ok = true;
}

void SchedulerComponent::GetCurrentCommand([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetCurrentCommand::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::GetCurrentCommand::Response>      response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    response->command = m_currentCommand;
    response->is_ok = true;
}

void SchedulerComponent::SetCommand([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::SetCommand::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::SetCommand::Response>      response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    std::string poi_name = m_tourStorage->GetTour().getPoIsList()[m_currentPoi];
    if(request->command.empty())
    {
        response->is_ok = false;
        response->error_msg = "Empty command field";
        return;
    }
    if(!checkIfCommandValid(poi_name, request->command))
    {
        response->is_ok = false;
        response->error_msg = "Command not available";
        return;
    }
    m_currentAction = 0;
    m_currentCommand = request->command;
    response->is_ok = true;
}

bool SchedulerComponent::checkIfCommandValid(const std::string &poiName, const std::string command)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
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
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    PoI currentPoi;
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    if(!m_tourStorage->GetTour().getPoI(poiName,currentPoi))
    {
        std::cout << "Error getting POI" << std::endl;
        return false;
    }
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    if(currentPoi.getActions(m_currentCommand, actions))
    {
	    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
        return true;
    }
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
    // Add if not in PoI search in General PoI
    PoI genericPoi;
    if(!m_tourStorage->GetTour().getPoI(GENERIC_POI_NAME,genericPoi))
    {
        std::cout << "Error getting Generic POI" << std::endl;
        return false;
    }
    if(genericPoi.getActions(m_currentCommand, actions))
    {
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
        return true;

    }
    std::cout << "Error getting Actions" << std::endl;
    return false;
}

void SchedulerComponent::GetAvailableCommands([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetAvailableCommands::Request> request,
             std::shared_ptr<scheduler_interfaces::srv::GetAvailableCommands::Response>      response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "SchedulerComponent " << __LINE__);
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
