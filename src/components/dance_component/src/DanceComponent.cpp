/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "DanceComponent.h"

bool DanceComponent::start(int argc, char *argv[])
{
    if (argc >= 2)
    {
        m_movementStorage = std::make_shared<MovementStorage>(); // Loads the movements json from the file and saves a reference to the class.
        if (!m_movementStorage->LoadMovements(argv[1]))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error loading movements file");
            return false;
        }
    }
    if (!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }

    m_node = rclcpp::Node::make_shared("DanceComponentNode");
    m_getMovementService = m_node->create_service<dance_interfaces::srv::GetMovement>("/DanceComponent/GetMovement",
                                                                                      std::bind(&DanceComponent::GetMovement,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));
    m_updateMovementService = m_node->create_service<dance_interfaces::srv::UpdateMovement>("/DanceComponent/UpdateMovement",
                                                                                            std::bind(&DanceComponent::UpdateMovement,
                                                                                                      this,
                                                                                                      std::placeholders::_1,
                                                                                                      std::placeholders::_2));
    m_setDanceService = m_node->create_service<dance_interfaces::srv::SetDance>("/DanceComponent/SetDance",
                                                                                std::bind(&DanceComponent::SetDance,
                                                                                          this,
                                                                                          std::placeholders::_1,
                                                                                          std::placeholders::_2));
    m_getDanceService = m_node->create_service<dance_interfaces::srv::GetDance>("/DanceComponent/GetDance",
                                                                                std::bind(&DanceComponent::GetDance,
                                                                                          this,
                                                                                          std::placeholders::_1,
                                                                                          std::placeholders::_2));
    m_getDanceDurationService = m_node->create_service<dance_interfaces::srv::GetDanceDuration>("/DanceComponent/GetDanceDuration",
                                                                                                std::bind(&DanceComponent::GetDanceDuration,
                                                                                                          this,
                                                                                                          std::placeholders::_1,
                                                                                                          std::placeholders::_2));
    m_getPartNamesService = m_node->create_service<dance_interfaces::srv::GetPartNames>("/DanceComponent/GetPartNames",
                                                                                        std::bind(&DanceComponent::GetPartNames,
                                                                                                  this,
                                                                                                  std::placeholders::_1,
                                                                                                  std::placeholders::_2));

    m_getBestDanceService = m_node->create_service<dance_interfaces::srv::GetBestDance>("/DanceComponent/GetBestDance",
                                                                                        std::bind(&DanceComponent::GetBestDance,
                                                                                                  this,
                                                                                                  std::placeholders::_1,
                                                                                                  std::placeholders::_2));

    RCLCPP_INFO(m_node->get_logger(), "DanceComponent::start");

    return true;
}

bool DanceComponent::close()
{
    rclcpp::shutdown();
    return true;
}

void DanceComponent::spin()
{
    rclcpp::spin(m_node);
}

void DanceComponent::GetMovement([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::GetMovement::Request> request,
                                 std::shared_ptr<dance_interfaces::srv::GetMovement::Response> response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "DanceComponent::GetMovement ");
    Dance dance;
    if (!m_movementStorage->GetMovementsContainer().GetDance(m_currentDance, dance))
    {
        response->is_ok = false;
        response->error_msg = "Dance not found";
        return;
    }
    response->part_name = dance.GetMovements()[m_currentMovement].GetPartName();
    response->time = dance.GetMovements()[m_currentMovement].GetTime();
    response->offset = dance.GetMovements()[m_currentMovement].GetOffset();
    response->joints = dance.GetMovements()[m_currentMovement].GetJoints();
    response->is_ok = true;
}

void DanceComponent::UpdateMovement([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::UpdateMovement::Request> request,
                                    std::shared_ptr<dance_interfaces::srv::UpdateMovement::Response> response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "DanceComponent::UpdateMovement ");
    Dance dance;
    if (!m_movementStorage->GetMovementsContainer().GetDance(m_currentDance, dance))
    {
        response->is_ok = false;
        response->error_msg = "Dance not found";
        return;
    }
    std::vector<Movement> movements_vec = dance.GetMovements();
    m_currentMovement = (m_currentMovement + 1);
    response->done_with_dance = false;
    if (m_currentMovement >= movements_vec.size())
    {
        response->done_with_dance = true;
        m_currentMovement = m_currentMovement % movements_vec.size();
    }
    response->is_ok = true;
}

void DanceComponent::SetDance([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::SetDance::Request> request,
                              std::shared_ptr<dance_interfaces::srv::SetDance::Response> response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "DanceComponent::SetDance name: " << request->dance);
    if (request->dance.empty())
    {
        response->is_ok = false;
        response->error_msg = "Empty dance field";
        return;
    }
    Dance dance;
    if (!m_movementStorage->GetMovementsContainer().GetDance(request->dance, dance))
    {
        response->is_ok = false;
        response->error_msg = "Dance not found";
        return;
    }
    m_currentMovement = 0;
    m_currentDance = request->dance;
    response->is_ok = true;
}

void DanceComponent::GetDance([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::GetDance::Request> request,
                              std::shared_ptr<dance_interfaces::srv::GetDance::Response> response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "DanceComponent::GetDance name: " << m_currentDance);
    response->dance = m_currentDance;
    response->is_ok = true;
}

void DanceComponent::GetDanceDuration([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::GetDanceDuration::Request> request,
                                      std::shared_ptr<dance_interfaces::srv::GetDanceDuration::Response> response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "DanceComponent::GetDanceDuration");
    Dance dance;
    if (!m_movementStorage->GetMovementsContainer().GetDance(m_currentDance, dance))
    {
        response->is_ok = false;
        response->error_msg = "Dance not found";
        return;
    }
    response->duration = dance.GetDuration();
    response->is_ok = true;
}

void DanceComponent::GetPartNames([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::GetPartNames::Request> request,
                                  std::shared_ptr<dance_interfaces::srv::GetPartNames::Response> response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "DanceComponent::GetPartNames");
    std::set<std::string> part_names = m_movementStorage->GetMovementsContainer().GetPartNames();
    for (auto part_name : part_names)
    {
        response->parts.push_back(part_name);
    }
    response->is_ok = true;
}

void DanceComponent::GetBestDance([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::GetBestDance::Request> request,
                                  std::shared_ptr<dance_interfaces::srv::GetBestDance::Response> response)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "DanceComponent::GetBestDance got request");

    std::map<std::string, Dance> dances = m_movementStorage->GetMovementsContainer().GetDances();

    float speechTime = request->speech_duration;
    std::string danceCategory = request->dance_category;

    float bestDanceSpeedFactor = std::numeric_limits<float>::max();
    std::string bestDance;
    float danceSpeedFactor;
    float bestDanceDuration = 0;
    bool danceExists = false;

    RCLCPP_INFO_STREAM(m_node->get_logger(), "DanceComponent::GetBestDance: Searching best dance for category " << danceCategory << " with duration " << speechTime);

    for (auto it = dances.begin(); it != dances.end(); it++)
    {
        std::string danceName = it->first;
        Dance dance = it->second;
        // if the dance name contains the dance category in its name, process the candidate
        if (danceName.find(danceCategory) != std::string::npos)
        {
            RCLCPP_INFO_STREAM(m_node->get_logger(), "DanceComponent::GetBestDance: Found candidate dance " << danceName);
            danceExists = true;
            danceSpeedFactor = speechTime / dance.GetDuration();
            if (std::abs(1 - danceSpeedFactor) <= bestDanceSpeedFactor)
            {
                bestDanceSpeedFactor = danceSpeedFactor;
                bestDance = danceName;
                bestDanceDuration = dance.GetDuration();
            }
        }
    }

    

    if (danceExists)
    {
        RCLCPP_INFO_STREAM(m_node->get_logger(), "DanceComponent::GetBestDance: Chosen best dance " << bestDance << " with duration " << bestDanceDuration);
        response->dance_name = bestDance;
        response->dance_duration = bestDanceDuration;
        response->is_ok = true;
    }
    else
    {
        RCLCPP_WARN_STREAM(m_node->get_logger(), "DanceComponent::GetBestDance: No dance found for category " << danceCategory);
        response->is_ok = false;
        response->error_msg = "Dance Category " + danceCategory + " not found";
    }
}