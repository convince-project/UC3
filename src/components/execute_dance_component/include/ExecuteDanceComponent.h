/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
# pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <dance_interfaces/srv/get_dance.hpp>
#include <dance_interfaces/srv/get_dance_duration.hpp>
#include <dance_interfaces/srv/get_movement.hpp>
#include <dance_interfaces/srv/set_dance.hpp>
#include <dance_interfaces/srv/get_part_names.hpp>
#include <dance_interfaces/srv/update_movement.hpp>
#include <execute_dance_interfaces/srv/execute_dance.hpp>
#include <execute_dance_interfaces/srv/is_dancing.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <map>

class ExecuteDanceComponent 
{
public:
    ExecuteDanceComponent() = default;
    bool start(int argc, char*argv[]);
    bool close();
    void spin();
    
    // ROS2 service handlers
    void ExecuteDance(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request,
                      std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Response> response);
    void IsDancing(const std::shared_ptr<execute_dance_interfaces::srv::IsDancing::Request> request,
                   std::shared_ptr<execute_dance_interfaces::srv::IsDancing::Response> response);

private:
    // Movement execution methods
    bool SendMovementToQueue(float time, int offset, std::vector<float> joints, yarp::os::Port &port);
    bool SendMovementNow(float time, int offset, std::vector<float> joints, yarp::os::Port &port);
    bool SendMovementToYAP(const std::string &actionName);

    // NEW: Pointing functionality methods
    bool ExecutePointingMovement(const std::shared_ptr<dance_interfaces::srv::GetMovement::Response> movement);
    std::string extractArtworkName(const std::vector<float>& joints);
    std::vector<double> transformMapToRobot(const std::vector<double>& mapCoords);
    bool sendCartesianCommand(const std::vector<double>& coords);

    // Task execution methods
    void executeTask(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request);
    void timerTask(float time);

    // Core component data
    std::string m_danceName;
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<execute_dance_interfaces::srv::ExecuteDance>::SharedPtr m_executeDanceService;
    rclcpp::Service<execute_dance_interfaces::srv::IsDancing>::SharedPtr m_isDancingService;
    std::map<std::string, yarp::os::Port &> m_pCtpService;
    std::thread m_threadExecute;
    std::thread m_threadTimer;
    std::mutex m_timerMutex;
    bool m_timerTask{false};

    // YARP Actions Players
    std::string yAPClientPortName;
    yarp::os::Port m_yAPClientPort;

    // AMCL localization subscription
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_amclSub;
    double m_currentX{0.0}, m_currentY{0.0}, m_currentYaw{0.0};

    // Points of Interest coordinates
    std::map<std::string, std::pair<double, double>> m_poiCoords;

    // YARP Cartesian controller
    yarp::dev::PolyDriver             m_cartesianClient;
    yarp::dev::ICartesianControl*     m_cartesianCtrl{nullptr};

    // Callback and helper methods
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    std::map<std::string, std::pair<double, double>> loadPoiCoordinates(const std::string& filename);
};
