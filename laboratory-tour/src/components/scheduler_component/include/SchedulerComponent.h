/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <scheduler_interfaces_dummy/srv/set_poi.hpp>
#include <scheduler_interfaces_dummy/srv/get_current_poi.hpp>

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
    void SetPoi([[maybe_unused]] const std::shared_ptr<scheduler_interfaces_dummy::srv::SetPoi::Request> request,
                std::shared_ptr<scheduler_interfaces_dummy::srv::SetPoi::Response>      response);
    void GetCurrentPoi([[maybe_unused]] const std::shared_ptr<scheduler_interfaces_dummy::srv::GetCurrentPoi::Request> request,
                std::shared_ptr<scheduler_interfaces_dummy::srv::GetCurrentPoi::Response>      response);

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<scheduler_interfaces_dummy::srv::SetPoi>::SharedPtr m_setPoiService;
    rclcpp::Service<scheduler_interfaces_dummy::srv::GetCurrentPoi>::SharedPtr m_getCurrentPoiService;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;

    bool checkIfCommandValid(const std::string &poiName, const std::string command);
    bool getActionsVector(const std::string &poiName, std::vector<Action> &actions);

    std::mutex m_mutex;
    int32_t m_currentPoi{0};
    int32_t m_currentAction{0};
    std::string m_currentCommand;
    std::shared_ptr<TourStorage> m_tourStorage;
};
