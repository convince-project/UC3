#pragma once

/******************************************************************************
 *                                                                            *
 * UC3 — ExecuteDanceComponent (header)                                        *
 *                                                                            *
 * Scopo: eseguire gesture di "pointing" verso opere/POI le cui coordinate    *
 * sono fornite nel frame `map`. Il componente:                                *
 *  - trasforma il target `map -> base_link`                                   *
 *  - sceglie il braccio in base alla Y nel frame `torso` (stile camera)       *
 *  - sposta la mano verso il target mantenendo orientazione corrente          *
 *  - invia il comando `go_to_pose` ai controller cartesiani YARP              *
 *  - strategia "camera-style": punto candidato sulla retta spalla→target      *
 *                                                                            *
 ******************************************************************************/

// ==== C++ standard ====
#include <mutex>
#include <thread>
#include <memory>
#include <map>
#include <vector>
#include <algorithm>

// ==== ROS2 base ====
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

// ==== YARP ====
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>

// ==== Algebra lineare (Eigen) ====
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ==== TF2 (buffer+listener e helpers per geometry_msgs) ====
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ==== Interfacce dei servizi ====
#include <execute_dance_interfaces/srv/execute_dance.hpp>
#include <execute_dance_interfaces/srv/is_dancing.hpp>

class ExecuteDanceComponent
{
public:
    ExecuteDanceComponent() = default;

    // Inizializza tutto (ROS2, TF, porte YARP, servizi)
    bool start(int argc, char* argv[]);
    // Rilascia risorse
    bool close();
    // Esegue lo spin ROS2 in questo thread (se vuoi)
    void spin();

private:
    // ======= Esecuzione del task (entrypoint del servizio ExecuteDance) =======
    void executeTask(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request);

    // ======= Callback ROS2 (es. AMCL) =======
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // ======= Caricamento configurazioni/coordinate =======
    std::map<std::string, std::vector<double>> loadArtworkCoordinates(const std::string& filename);

    bool preScanArticulatedArms(const Eigen::Vector3d& artwork_pos,
                                std::vector<std::pair<std::string,double>>& armDistances,
                                std::map<std::string, std::vector<double>>& cachedPoseValues);

    bool isPoseReachable(yarp::os::Port* activePort,
                         const Eigen::Vector3d& candidate,
                         const Eigen::Quaterniond& q_target);

    // ======= Helper TF (map -> robot) =======
    bool transformPointMapToRobot(const geometry_msgs::msg::Point& map_point,
                                  geometry_msgs::msg::Point& out_robot_point,
                                  const std::string& robot_frame,
                                  double timeout_sec);

    // ======= TF2 persistente =======
    std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tfListener;

    // Nomi dei frame
    std::string m_mapFrame        {"map"};
    std::string m_baseFrame       {"mobile_base_body_link"};
    std::string m_torsoFrame      {"chest_link"};
    std::string m_lShoulderFrame  {"l_shoulder_1"};
    std::string m_rShoulderFrame  {"r_shoulder_1"};

    // Parametri pointing
    double m_reachRadius   {0.60};
    double m_minDist       {0.40};
    double m_safetyBackoff {0.05};

    enum class ToolAxis { AlignX, AlignZ };
    ToolAxis m_toolAxis { ToolAxis::AlignZ }; // fissa: Z → target

    // ---- TF helpers ----
    bool getTFMatrix(const std::string& target,
                     const std::string& source,
                     Eigen::Matrix4d& T) const;
    static Eigen::Vector3d transformPoint(const Eigen::Matrix4d& T,
                                          const Eigen::Vector3d& p);

    bool chooseArmByTorsoY(const Eigen::Vector3d& p_base,
                           std::string& outArm) const;
    bool getShoulderPosInBase(const std::string& arm,
                              Eigen::Vector3d& out_pos) const;

    Eigen::Quaterniond quatAlignAxisToDir(const Eigen::Vector3d& dir_base,
                                          ToolAxis axis,
                                          const Eigen::Vector3d& worldUp = Eigen::Vector3d::UnitZ()) const;
    Eigen::Vector3d sphereReachPoint(const Eigen::Vector3d& shoulder_base,
                                     const Eigen::Vector3d& target_base) const;

private:
    // ======= Stato runtime =======
    const std::string cartesianPortLeft  = "/r1-cartesian-control/left_arm/rpc:i";
    const std::string cartesianPortRight = "/r1-cartesian-control/right_arm/rpc:i";

    std::string m_danceName;
    rclcpp::Node::SharedPtr m_node;

    rclcpp::Service<execute_dance_interfaces::srv::ExecuteDance>::SharedPtr m_executeDanceService;
    rclcpp::Service<execute_dance_interfaces::srv::IsDancing>::SharedPtr    m_isDancingService;

    double m_currentX{0.0};
    double m_currentY{0.0};
    double m_currentYaw{0.0};

    std::map<std::string, std::vector<double>> m_artworkCoords;

    yarp::dev::PolyDriver m_cartesianClient;
    std::string m_cartesianPortNameLeft  = "/ExecuteDanceComponent/cartesianClientLeft/rpc";
    std::string m_cartesianPortNameRight = "/ExecuteDanceComponent/cartesianClientRight/rpc";
    yarp::os::Port m_cartesianPortLeft;
    yarp::os::Port m_cartesianPortRight;

    std::string cartesianControllerIniPathLeft  =
        "/home/user1/ergocub-cartesian-control/src/r1_cartesian_control/app/conf/config_left_sim_r1.ini";
    std::string cartesianControllerIniPathRight =
        "/home/user1/ergocub-cartesian-control/src/r1_cartesian_control/app/conf/config_right_sim_r1.ini";
};
