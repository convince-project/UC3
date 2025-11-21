/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <scheduler_interfaces/srv/update_poi.hpp>
#include <scheduler_interfaces/srv/reset.hpp>
#include <scheduler_interfaces/srv/end_tour.hpp>
#include <scheduler_interfaces/srv/get_current_poi.hpp>
#include <scheduler_interfaces/srv/update_action.hpp>
#include <scheduler_interfaces/srv/get_current_action.hpp>
#include <scheduler_interfaces/srv/set_language.hpp>
#include <scheduler_interfaces/srv/get_current_language.hpp>
#include <scheduler_interfaces/srv/set_command.hpp>
#include <scheduler_interfaces/srv/get_current_command.hpp>
#include <scheduler_interfaces/srv/get_available_commands.hpp>
#include <scheduler_interfaces/srv/set_poi.hpp>
#include <cmath>

#include <map>
#include "TourStorage.h"

#define GENERIC_POI_NAME "___generic___"

class SchedulerComponent
{
public:
    SchedulerComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    void publisher(std::string text);
    void UpdatePoi([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::UpdatePoi::Request> request,
                std::shared_ptr<scheduler_interfaces::srv::UpdatePoi::Response>      response);
    void GetCurrentPoi([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetCurrentPoi::Request> request,
                std::shared_ptr<scheduler_interfaces::srv::GetCurrentPoi::Response>      response);
    void Reset( [[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::Reset::Request> request,
                std::shared_ptr<scheduler_interfaces::srv::Reset::Response>      response);
    void EndTour( [[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::EndTour::Request> request,
                std::shared_ptr<scheduler_interfaces::srv::EndTour::Response>      response);
    void UpdateAction([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::UpdateAction::Request> request,
                std::shared_ptr<scheduler_interfaces::srv::UpdateAction::Response>      response);
    void GetCurrentAction([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetCurrentAction::Request> request,
                std::shared_ptr<scheduler_interfaces::srv::GetCurrentAction::Response>      response);
    void GetCurrentLanguage([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetCurrentLanguage::Request> request,
                std::shared_ptr<scheduler_interfaces::srv::GetCurrentLanguage::Response>      response);
    void SetLanguage([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::SetLanguage::Request> request,
                std::shared_ptr<scheduler_interfaces::srv::SetLanguage::Response>      response);
    void GetCurrentCommand([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetCurrentCommand::Request> request,
                std::shared_ptr<scheduler_interfaces::srv::GetCurrentCommand::Response>      response);
    void SetCommand([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::SetCommand::Request> request,
                std::shared_ptr<scheduler_interfaces::srv::SetCommand::Response>      response);
    void GetAvailableCommands([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetAvailableCommands::Request> request,
                std::shared_ptr<scheduler_interfaces::srv::GetAvailableCommands::Response>      response);
    void SetPoi([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::SetPoi::Request> request,
                std::shared_ptr<scheduler_interfaces::srv::SetPoi::Response>      response);
    void SetPoiFromPlanner([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::SetPoi::Request> request,
                std::shared_ptr<scheduler_interfaces::srv::SetPoi::Response>      response);
    void GetCurrentPoiForNavigation([[maybe_unused]] const std::shared_ptr<scheduler_interfaces::srv::GetCurrentPoi::Request> request,
                std::shared_ptr<scheduler_interfaces::srv::GetCurrentPoi::Response>      response);

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<scheduler_interfaces::srv::UpdatePoi>::SharedPtr m_updatePoiService;
    rclcpp::Service<scheduler_interfaces::srv::GetCurrentPoi>::SharedPtr m_getCurrentPoiService;
    rclcpp::Service<scheduler_interfaces::srv::Reset>::SharedPtr m_resetService;
    rclcpp::Service<scheduler_interfaces::srv::EndTour>::SharedPtr m_endTourService;
    rclcpp::Service<scheduler_interfaces::srv::UpdateAction>::SharedPtr m_updateActionService;
    rclcpp::Service<scheduler_interfaces::srv::GetCurrentAction>::SharedPtr m_getCurrentActionService;
    rclcpp::Service<scheduler_interfaces::srv::GetCurrentLanguage>::SharedPtr m_getCurrentLanguageService;
    rclcpp::Service<scheduler_interfaces::srv::SetLanguage>::SharedPtr m_setLanguageService;
    rclcpp::Service<scheduler_interfaces::srv::GetCurrentCommand>::SharedPtr m_getCurrentCommandService;
    rclcpp::Service<scheduler_interfaces::srv::SetCommand>::SharedPtr m_setCommandService;
    rclcpp::Service<scheduler_interfaces::srv::GetAvailableCommands>::SharedPtr m_getAvailableCommandsService;
    rclcpp::Service<scheduler_interfaces::srv::SetPoi>::SharedPtr m_setPoiService;
    rclcpp::Service<scheduler_interfaces::srv::GetCurrentPoi>::SharedPtr m_getCurrentPoiForNavigationService;
    rclcpp::Service<scheduler_interfaces::srv::SetPoi>::SharedPtr m_setPoiFromPlannerService;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;


    bool checkIfCommandValid(const std::string &poiName, const std::string command);
    bool getActionsVector(const std::string &poiName, std::vector<Action> &actions);

    std::mutex m_mutex;
    int32_t m_currentPoi{0};
    int32_t m_currentAction{0};
    int32_t m_alternative_poi{false};
    std::string m_currentCommand;
    std::shared_ptr<TourStorage> m_tourStorage;
};
