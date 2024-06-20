/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <allowed_to_move_interfaces/srv/is_allowed_to_move.hpp>
#include <allowed_to_move_interfaces/srv/set_allowed_to_move.hpp>

class AllowedToMoveComponent
{
public:
    AllowedToMoveComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    void IsAllowedToMove([[maybe_unused]] const std::shared_ptr<allowed_to_move_interfaces::srv::IsAllowedToMove::Request> request,
                std::shared_ptr<allowed_to_move_interfaces::srv::IsAllowedToMove::Response>      response);
    void SetAllowedToMove([[maybe_unused]] const std::shared_ptr<allowed_to_move_interfaces::srv::SetAllowedToMove::Request> request,
                std::shared_ptr<allowed_to_move_interfaces::srv::SetAllowedToMove::Response>      response);

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<allowed_to_move_interfaces::srv::IsAllowedToMove>::SharedPtr m_isAllowedToMoveService;
    rclcpp::Service<allowed_to_move_interfaces::srv::SetAllowedToMove>::SharedPtr m_setAllowedToMoveService;
    std::atomic<bool> m_allowedToMove;
};
