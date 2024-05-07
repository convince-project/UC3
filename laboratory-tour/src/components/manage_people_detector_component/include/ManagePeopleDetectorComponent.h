/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <manage_service_interfaces/srv/start_service.hpp>
#include <manage_service_interfaces/srv/stop_service.hpp>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp" 
#include <optional>

using namespace std::chrono_literals;
class ManagePeopleDetectorComponent
{
public:
    ManagePeopleDetectorComponent() = default;

    bool start(int argc, char*argv[]);

    bool close();
    void spin();
    void StopService([[maybe_unused]] const std::shared_ptr<manage_service_interfaces::srv::StopService::Request> request,
                std::shared_ptr<manage_service_interfaces::srv::StopService::Response>      response);
    void StartService( [[maybe_unused]] const std::shared_ptr<manage_service_interfaces::srv::StartService::Request> request,
                std::shared_ptr<manage_service_interfaces::srv::StartService::Response>      response);

private:
    std::optional<lifecycle_msgs::msg::State> get_state(const std::chrono::seconds& timeout = 3s);
    bool is_client_available(const std::chrono::seconds& timeout = 3s);
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<manage_service_interfaces::srv::StopService>::SharedPtr m_stopServiceService;
    rclcpp::Service<manage_service_interfaces::srv::StartService>::SharedPtr m_startServiceService;
    std::mutex m_mutex;
    std::shared_ptr<std::thread> m_thread;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr m_client_get_state;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr m_client_change_state;
    std::string m_name = "StartServicesDataModel";
    std::chrono::seconds m_timeout = 3s;
    std::optional<lifecycle_msgs::msg::State> m_currentState;

};
