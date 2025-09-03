#pragma once

/******************************************************************************
 *                                                                            *
 * UC3 — ExecuteDanceComponent (header)                                        *
 *                                                                            *
 * Scopo: eseguire gesture di "pointing" verso opere/POI le cui coordinate    *
 * sono fornite nel frame `map`. Il componente:                                *
 *  - trasforma il target `map -> base_link`                                   *
 *  - sceglie il braccio in base alla Y nel frame `torso` (stile camera)       *
 *  - costruisce l'orientazione dell'end-effector allineando **X** al target   *
 *  - invia il comando `go_to_pose` ai controller cartesiani YARP              *
 *  - strategia "camera-style": punto candidato sulla retta spalla→target      *
 *    (niente ricerca binaria)                                                 *
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

// ==== Interfacce dei servizi (snake_case corretti) ====
#include <execute_dance_interfaces/srv/execute_dance.hpp>
#include <execute_dance_interfaces/srv/is_dancing.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp> // se in futuro dovesse servire array di marker

/**
 * @class ExecuteDanceComponent
 * @brief Esegue gesture di pointing verso un target 3D definito in `map`.
 *
 * Design:
 *  - TF2 persistente (buffer+listener) come membri di classe.
 *  - Scelta braccio stile camera (segno della y in `torso`).
 *  - Orientazione end-effector allineata all'asse **X** verso il target (AlignX fisso).
 *  - Candidato iniziale su retta spalla→target con raggio limitato (niente binary search).
 */
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

    // ======= Reachability & motion helpers (RPC) =======
    bool checkPoseReachabilityForArm(double x, double y, double z, const std::string& armName);
    bool checkPoseReachability(double x, double y, double z);

    // Pre-scan: interroga i controller per ottenere la posa mano e distanza dal target
    bool preScanArticulatedArms(const Eigen::Vector3d& artwork_pos,
                                std::vector<std::pair<std::string,double>>& armDistances,
                                std::map<std::string, std::vector<double>>& cachedPoseValues);

    // Wrapper RPC per `is_pose_reachable`
    bool isPoseReachable(yarp::os::Port* activePort,
                         const Eigen::Vector3d& candidate,
                         const Eigen::Quaterniond& q_target);

    // ======= Helper TF (map -> robot) usando il buffer persistente =======
    bool transformPointMapToRobot(const geometry_msgs::msg::Point& map_point,
                                  geometry_msgs::msg::Point& out_robot_point,
                                  const std::string& robot_frame,
                                  double timeout_sec);

    // === RViz Markers ===
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_markerPub;

    // Helpers per i marker
    void publishMarkerSphere(const std::string& ns, int id,
                            const Eigen::Vector3d& p,
                            const std::string& frame_id,
                            float r, float g, float b,
                            float scale = 0.07f, float alpha = 1.0f) const;

    void publishMarkerArrow(const std::string& ns, int id,
                            const Eigen::Vector3d& p_from,
                            const Eigen::Vector3d& p_to,
                            const std::string& frame_id,
                            float r, float g, float b,
                            float shaft_diam = 0.02f, float head_diam = 0.04f, float head_len = 0.06f,
                            float alpha = 1.0f) const;

    void deleteAllMarkers() const;

    // Disegna l'asse X dell'EEF come freccia partendo da 'origin' usando l'orientazione q
    void publishEefXAxis(const std::string& ns, int id,
                        const Eigen::Vector3d& origin,
                        const Eigen::Quaterniond& q,
                        const std::string& frame_id,
                        double length = 0.20,
                        float r = 1.0f, float g = 0.2f, float b = 0.2f, // rosso
                        float shaft_diam = 0.02f, float head_diam = 0.05f, float head_len = 0.08f,
                        float alpha = 1.0f) const;

    // ======= Nuovi helper unificati + TF2 persistente =======
    // TF2 (buffer+listener) come membri per non ricreare listener ad ogni lookup
    std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tfListener;

    // Nomi dei frame (override da config se vuoi)
    std::string m_mapFrame        {"map"};
    std::string m_baseFrame       {"mobile_base_body_link"};
    std::string m_torsoFrame      {"chest_link"};
    std::string m_lShoulderFrame  {"l_shoulder_1"};
    std::string m_rShoulderFrame  {"r_shoulder_1"};

    // Parametri di pointing (tarabili)
    double m_reachRadius   {0.60};   // raggio max su cui proiettare il candidato dalla spalla
    double m_minDist       {0.40};   // distanza minima
    double m_safetyBackoff {0.05};   // non arrivare a toccare il target

    // Quale asse dell'EEF punta verso il target (fisso a AlignZ)
    enum class ToolAxis { AlignX, AlignZ };
    ToolAxis m_toolAxis { ToolAxis::AlignZ }; // fisso:  Z → target

    // ---- TF helpers ----
    bool getTFMatrix(const std::string& target,
                     const std::string& source,
                     Eigen::Matrix4d& T) const; // lookupTransform e costruzione di T omogenea
    static Eigen::Vector3d transformPoint(const Eigen::Matrix4d& T,
                                          const Eigen::Vector3d& p); // T*[p;1]

    // ---- Target/arm helpers ----
    bool chooseArmByTorsoY(const Eigen::Vector3d& p_base,
                           std::string& outArm) const; // "LEFT" o "RIGHT" in base al segno della Y in torso
    bool getShoulderPosInBase(const std::string& arm,
                              Eigen::Vector3d& out_pos) const;   // posizione spalla nel frame base
    Eigen::Quaterniond quatAlignAxisToDir(const Eigen::Vector3d& dir_base,
                                          ToolAxis axis,
                                          const Eigen::Vector3d& worldUp = Eigen::Vector3d::UnitZ()) const; // costruisce R con X verso dir
    Eigen::Vector3d sphereReachPoint(const Eigen::Vector3d& shoulder_base,
                                     const Eigen::Vector3d& target_base) const; // candidato iniziale sulla retta spalla→target

private:
    // ======= Stato runtime =======

    // Porte RPC dei controller cartesiani (etichette comode)
    const std::string cartesianPortLeft  = "/r1-cartesian-control/left_arm/rpc:i";
    const std::string cartesianPortRight = "/r1-cartesian-control/right_arm/rpc:i";

    std::string m_danceName;                          // nome dell'opera richiesto
    rclcpp::Node::SharedPtr m_node;                   // nodo ROS2

    // Servizi ROS2 (qui: ExecuteDance)
    rclcpp::Service<execute_dance_interfaces::srv::ExecuteDance>::SharedPtr m_executeDanceService;
    rclcpp::Service<execute_dance_interfaces::srv::IsDancing>::SharedPtr    m_isDancingService;

    // (opzionale) stato di localizzazione corrente (es. AMCL)
    double m_currentX{0.0};
    double m_currentY{0.0};
    double m_currentYaw{0.0};

    // Dizionari di coordinate delle opere
    std::map<std::string, std::vector<double>> m_artworkCoords;

    // Porte RPC verso i controller cartesiani (LEFT/RIGHT)
    yarp::dev::PolyDriver m_cartesianClient; // compatibilità, non usato direttamente
    std::string m_cartesianPortNameLeft  = "/ExecuteDanceComponent/cartesianClientLeft/rpc";
    std::string m_cartesianPortNameRight = "/ExecuteDanceComponent/cartesianClientRight/rpc";
    yarp::os::Port m_cartesianPortLeft;
    yarp::os::Port m_cartesianPortRight;

    // Percorsi dei file ini dei controller (modifica per il tuo setup)
    std::string cartesianControllerIniPathLeft  =
        "/home/user1/ergocub-cartesian-control/src/r1_cartesian_control/app/conf/config_left_sim_r1.ini";
    std::string cartesianControllerIniPathRight =
        "/home/user1/ergocub-cartesian-control/src/r1_cartesian_control/app/conf/config_right_sim_r1.ini";
};