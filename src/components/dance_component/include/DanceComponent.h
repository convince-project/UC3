/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <dance_interfaces/srv/get_movement.hpp>
#include <dance_interfaces/srv/update_movement.hpp>
#include <dance_interfaces/srv/set_dance.hpp>
#include <dance_interfaces/srv/get_dance.hpp>
#include <dance_interfaces/srv/get_dance_duration.hpp>
#include <dance_interfaces/srv/get_part_names.hpp>
#include "movementStorage.h"
#include "dance.h"


class DanceComponent
{
public:
    DanceComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    void GetMovement([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::GetMovement::Request> request,
                std::shared_ptr<dance_interfaces::srv::GetMovement::Response>      response);
    void UpdateMovement([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::UpdateMovement::Request> request,
                std::shared_ptr<dance_interfaces::srv::UpdateMovement::Response>      response);
    void SetDance([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::SetDance::Request> request,
                std::shared_ptr<dance_interfaces::srv::SetDance::Response>      response);
    void GetDance([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::GetDance::Request> request,
                std::shared_ptr<dance_interfaces::srv::GetDance::Response>      response);
    void GetDanceDuration([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::GetDanceDuration::Request> request,
                std::shared_ptr<dance_interfaces::srv::GetDanceDuration::Response>      response);
    void GetPartNames([[maybe_unused]] const std::shared_ptr<dance_interfaces::srv::GetPartNames::Request> request,
                std::shared_ptr<dance_interfaces::srv::GetPartNames::Response>      response);


private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<dance_interfaces::srv::GetMovement>::SharedPtr m_getMovementService;
    rclcpp::Service<dance_interfaces::srv::UpdateMovement>::SharedPtr m_updateMovementService;
    rclcpp::Service<dance_interfaces::srv::SetDance>::SharedPtr m_setDanceService;
    rclcpp::Service<dance_interfaces::srv::GetDance>::SharedPtr m_getDanceService;
    rclcpp::Service<dance_interfaces::srv::GetDanceDuration>::SharedPtr m_getDanceDurationService;
    rclcpp::Service<dance_interfaces::srv::GetPartNames>::SharedPtr m_getPartNamesService;

    // std::mutex m_mutex; actually not used
    int32_t m_currentMovement{0};
    std::string m_currentDance;
    std::shared_ptr<MovementStorage> m_movementStorage;
};
