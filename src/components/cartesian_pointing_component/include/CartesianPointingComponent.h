#pragma once
/******************************************************************************
 * CartesianPointingComponent (header)
 *
 * Variant minima: usa YARP ResourceFinder **solo** per trovare il file
 * `artwork_coords.json` nel context (default: "r1_cartesian_control").
 * Il resto del comportamento resta invariato.
 ******************************************************************************/

// C++
#include <mutex>
#include <thread>
#include <memory>
#include <map>
#include <vector>
#include <string>
#include <algorithm>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// YARP
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/ResourceFinder.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Interfaces
#include <cartesian_pointing_interfaces/srv/point_at.hpp>
#include <cartesian_pointing_interfaces/srv/is_pointing.hpp>

class CartesianPointingComponent
{
public:
    CartesianPointingComponent() = default;

    // Initialize ROS2, TF, YARP ports and services
    bool start(int argc, char* argv[]);
    bool close();
    void spin();

private:
    // Main operation
    void pointTask(const std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Request> request);

    // Config / data loading
    std::map<std::string, std::vector<double>>
    loadArtworkCoordinates(const std::string& filename);

    // Pre-scan & reachability
    bool preScanArticulatedArms(const Eigen::Vector3d& artwork_pos,
                                std::vector<std::pair<std::string,double>>& armDistances,
                                std::map<std::string, std::vector<double>>& cachedPoseValues);

    bool isPoseReachable(yarp::os::Port* activePort,
                         const Eigen::Vector3d& candidate,
                         const Eigen::Quaterniond& q_target);

    // TF helpers
    bool transformPointMapToRobot(const geometry_msgs::msg::Point& map_point,
                                  geometry_msgs::msg::Point& out_robot_point,
                                  const std::string& robot_frame,
                                  double timeout_sec);

    bool getTFMatrix(const std::string& target,
                     const std::string& source,
                     Eigen::Matrix4d& T) const;

    static Eigen::Vector3d transformPoint(const Eigen::Matrix4d& T,
                                          const Eigen::Vector3d& p);

    bool chooseArmByTorsoY(const Eigen::Vector3d& p_base,
                           std::string& outArm) const;

    bool getShoulderPosInBase(const std::string& arm,
                              Eigen::Vector3d& out_pos) const;

    Eigen::Vector3d sphereReachPoint(const Eigen::Vector3d& shoulder_base,
                                     const Eigen::Vector3d& target_base) const;

private:
    // ROS2
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<cartesian_pointing_interfaces::srv::PointAt>::SharedPtr   m_srvPointAt;
    rclcpp::Service<cartesian_pointing_interfaces::srv::IsPointing>::SharedPtr m_srvIsPointing;

    // TF2
    std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tfListener;

    // Frames
    std::string m_mapFrame        {"map"};
    std::string m_baseFrame       {"mobile_base_body_link"};
    std::string m_torsoFrame      {"chest_link"};
    std::string m_lShoulderFrame  {"l_shoulder_1"};
    std::string m_rShoulderFrame  {"r_shoulder_1"};

    // Pointing parameters
    double m_reachRadius   {0.60};
    double m_minDist       {0.40};
    double m_safetyBackoff {0.05};

    // YARP controller ports (server)
    const std::string cartesianPortLeft  = "/r1-cartesian-control/left_arm/rpc:i";
    const std::string cartesianPortRight = "/r1-cartesian-control/right_arm/rpc:i";

    // Local client port names (this component)
    std::string m_cartesianPortNameLeft  = "/CartesianPointingComponent/cartesianClientLeft/rpc";
    std::string m_cartesianPortNameRight = "/CartesianPointingComponent/cartesianClientRight/rpc";

    yarp::os::Port m_cartesianPortLeft;
    yarp::os::Port m_cartesianPortRight;

    // (Se vuoi usare i PolyDriver in futuro)
    yarp::dev::PolyDriver m_cartesianClient;

    // Percorsi .ini dei controller (lasciati come prima)
    std::string cartesianControllerIniPathLeft  =
        "/home/user1/ergocub-cartesian-control/src/r1_cartesian_control/app/conf/config_left_sim_r1.ini";
    std::string cartesianControllerIniPathRight =
        "/home/user1/ergocub-cartesian-control/src/r1_cartesian_control/app/conf/config_right_sim_r1.ini";

    // Artwork targets
    std::map<std::string, std::vector<double>> m_artworkCoords;

    // Runtime flags
    std::mutex m_flagMutex;
    bool m_isPointing {false};

    // ResourceFinder defaults (SOLO per artworks)
    std::string m_rfDefaultContext  {"r1_cartesian_control"};
    std::string m_rfDefaultArtworks {"artwork_coords.json"};
};
