/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "execute_dance_interfaces/srv/execute_dance.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yarp/os/Port.h>
#include <map>
#include <vector>
#include <string>
#include <yarp/os/Port.h>
#include <memory>
#include <tuple>

/**
 * Minimal ExecuteDanceComponent header — shoulder-only pointing, YARP Cartesian controller only.
 */
class ExecuteDanceComponent
{
public:
    ExecuteDanceComponent() = default;
    bool start(int argc, char* argv[]);
    bool close();
    void spin();

private:
    // Core task
    void executeTask(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request);

    // Utilities
    std::map<std::string, std::vector<double>> loadArtworkCoordinates(const std::string& filename);
    bool preScanArticulatedArms(const Eigen::Vector3d& artwork_pos,
                                std::vector<std::pair<std::string,double>>& armDistances,
                                std::map<std::string, std::vector<double>>& /*cachedPoseValues*/);

    // TF2 for shoulder lookups
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
    std::string m_r_shoulder_frame_id = "r_shoulder_link";
    std::string m_l_shoulder_frame_id = "l_shoulder_link";

    // ROS2 node + service
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<execute_dance_interfaces::srv::ExecuteDance>::SharedPtr m_executeDanceService;

    // YARP Cartesian controller ports (we always send to cartesian controller)
    yarp::os::Port m_cartesianPortLeft;
    yarp::os::Port m_cartesianPortRight;

    // Artworks
    std::map<std::string, std::vector<double>> m_artworkCoords;

    // Parameters
    double m_reach_radius = 0.6;

    // Pointing helpers (shoulder-based)
    Eigen::Vector3d reachablePointSphere(const Eigen::Vector3d& shoulder,
                                         const Eigen::Vector3d& target,
                                         double reach_radius) const;
    Eigen::Matrix3d getAlignedRotation(const Eigen::Vector3d& v) const;
    std::tuple<double,double,double,double> rot2quats(const Eigen::Matrix3d& R) const;

    bool isPoseReachable(yarp::os::Port* activePort,
                         const Eigen::Vector3d& candidate,
                         const Eigen::Quaterniond& q_target);

    bool probeBinarySearch(yarp::os::Port* activePort,
                           const Eigen::Vector3d& start_pos,
                           const Eigen::Vector3d& end_pos,
                           const Eigen::Vector3d& dir,
                           double dir_norm,
                           const Eigen::Quaterniond& q_target,
                           Eigen::Vector3d& out_best_candidate);

    // Local names for YARP client ports (used by start())
    std::string yAPClientPortName;
    yarp::os::Port m_yAPClientPort;

    std::string m_cartesianPortNameLeft;
    std::string m_cartesianPortNameRight;

    // Paths for cartesian controller INI files (can be overridden via env or CLI)
    std::string cartesianControllerIniPathLeft;
    std::string cartesianControllerIniPathRight;

    // Note: m_cartesianPortLeft / m_cartesianPortRight likely already declared as yarp::os::Port
};
