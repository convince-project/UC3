/******************************************************************************
 *                                                                            *
 * UC3 — ExecuteDanceComponent (header)                                        *
 *                                                                            *
 * Scopo: eseguire gesture di "pointing" verso opere/POI le cui coordinate   *
 * sono fornite nel frame `map`. Il componente:                               *
 *  - trasforma il target `map -> base_link`                                   *
 *  - sceglie il braccio in base alla Y nel frame `torso` (come il thread     *
 *    con la camera)                                                          *
 *  - costruisce l'orientazione dell'end-effector allineando X o Z al target  *
 *  - invia il comando `go_to_pose` ai controller cartesiani YARP              *
 *                                                                            *
 ******************************************************************************/
#pragma once

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

// ==== YARP ====
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>

// ==== Algebra lineare (Eigen) ====
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ==== TF2 (buffer+listener e helpers per geometry_msgs) ====
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ==== Interfacce dei servizi (già nel tuo workspace) ====
#include <execute_dance_interfaces/srv/execute_dance.hpp>
#include <execute_dance_interfaces/srv/is_dancing.hpp>

/**
 * @class ExecuteDanceComponent
 * @brief Esegue gesture di pointing verso un target 3D definito in `map`.
 *
 * Design:
 *  - TF2 persistente (buffer+listener) come membri di classe.
 *  - Scelta braccio stile thread camera (segno della y in `torso`).
 *  - Orientazione end-effector allineata all'asse desiderato (X o Z) verso il target.
 *  - Candidato iniziale su sfera shoulder→target + fallback con binary search
 *    lungo la retta mano→target usando `is_pose_reachable` del controller.
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
    // ======= Invio di azioni all'Action Player (se lo usi nella pipeline) =======
    bool SendMovementToYAP(const std::string &actionName, float speedFactor);

    // ======= Esecuzione del task (entrypoint del servizio ExecuteDance) =======
    void executeTask(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request);
    void timerTask(float time);

    // ======= Callback ROS2 (es. AMCL) =======
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // ======= Caricamento configurazioni/coordinate =======
    std::map<std::string, std::pair<double,double>> loadPoiCoordinates(const std::string& filename);
    std::map<std::string, std::vector<double>>      loadArtworkCoordinates(const std::string& filename);

    // ======= Setup YARP (se serve usare ResourceFinder) =======
    bool ConfigureYARP(yarp::os::ResourceFinder &rf);

    // ======= Reachability & motion helpers (RPC) =======
    bool checkPoseReachabilityForArm(double x, double y, double z, const std::string& armName);
    bool sendPositionCommand(double x, double y, double z, const std::string& armName);
    bool checkPoseReachability(double x, double y, double z);

    // Pre-scan: interroga i controller per ottenere la posa mano e distanza dal target
    bool preScanArticulatedArms(const Eigen::Vector3d& artwork_pos,
                                std::vector<std::pair<std::string,double>>& armDistances,
                                std::map<std::string, std::vector<double>>& cachedPoseValues);

    // (Legacy) orientazione basata sulla posa mano corrente (asse X → target)
    bool computeOrientationFromFlatPose(const std::vector<double>& flat_pose,
                                        const Eigen::Vector3d& artwork_pos,
                                        Eigen::Vector3d& hand_pos,
                                        Eigen::Vector3d& vec_to_artwork,
                                        double& vec_norm,
                                        Eigen::Matrix3d& R_hand,
                                        Eigen::Quaterniond& q_target);

    // Wrapper RPC per `is_pose_reachable`
    bool isPoseReachable(yarp::os::Port* activePort,
                         const Eigen::Vector3d& candidate,
                         const Eigen::Quaterniond& q_target);

    // Ricerca binaria lungo mano→target per trovare il punto migliore raggiungibile
    bool probeBinarySearch(yarp::os::Port* activePort,
                           const Eigen::Vector3d& hand_pos,
                           const Eigen::Vector3d& artwork_pos,
                           const Eigen::Vector3d& vec_to_artwork,
                           double vec_norm,
                           const Eigen::Quaterniond& q_target,
                           Eigen::Vector3d& out_best_candidate);

    // ======= Helper TF legacy (map -> robot) per compatibilità =======
    bool transformPointMapToRobot(const geometry_msgs::msg::Point& map_point,
                                  geometry_msgs::msg::Point& out_robot_point,
                                  const std::string& robot_frame,
                                  double timeout_sec);

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
    double m_minDist       {0.40};   // distanza minima (evita singularità molto vicine)
    double m_safetyBackoff {0.05};   // non arrivare a toccare il target

    // Quale asse dell'EEF punta verso il target: X (stile vecchio) o Z (stile camera)
    enum class ToolAxis { AlignX, AlignZ };
    ToolAxis m_toolAxis { ToolAxis::AlignZ }; // default: come `HandPointingThread`

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
                                          const Eigen::Vector3d& worldUp = Eigen::Vector3d::UnitY()) const; // costruisce R con asse desiderato verso dir
    Eigen::Vector3d sphereReachPoint(const Eigen::Vector3d& shoulder_base,
                                     const Eigen::Vector3d& target_base) const; // candidato iniziale sulla retta spalla→target

private:
    // ======= Stato runtime =======
    std::string m_danceName;                          // nome dell'opera richiesto
    rclcpp::Node::SharedPtr m_node;                   // nodo ROS2

    // Servizi ROS2 (qui: ExecuteDance)
    rclcpp::Service<execute_dance_interfaces::srv::ExecuteDance>::SharedPtr m_executeDanceService;
    rclcpp::Service<execute_dance_interfaces::srv::IsDancing>::SharedPtr    m_isDancingService;

    // Thread ausiliari
    std::thread m_threadExecute;
    std::thread m_threadTimer;
    std::mutex  m_timerMutex;
    bool        m_timerTask{false};

    // YARP Action Player client
    std::string   yAPClientPortName;
    yarp::os::Port m_yAPClientPort;

    // (opzionale) stato di localizzazione corrente (es. AMCL)
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_amclSub;
    double m_currentX{0.0};
    double m_currentY{0.0};
    double m_currentYaw{0.0};

    // Dizionari di POI/opere
    std::map<std::string, std::pair<double, double>> m_poiCoords;
    std::map<std::string, std::vector<double>>       m_artworkCoords;

    // Porte RPC verso i controller cartesiani (LEFT/RIGHT)
    yarp::dev::PolyDriver m_cartesianClient; // compatibilità, non usato direttamente
    std::string m_cartesianPortNameLeft;
    std::string m_cartesianPortNameRight;
    yarp::os::Port m_cartesianPortLeft;
    yarp::os::Port m_cartesianPortRight;

    // Percorsi dei file ini dei controller (modifica per il tuo setup)
    std::string cartesianControllerIniPathLeft  = 
        "/home/user1/ergocub-cartesian-control/src/r1_cartesian_control/app/conf/config_left_sim_r1.ini";
    std::string cartesianControllerIniPathRight = 
        "/home/user1/ergocub-cartesian-control/src/r1_cartesian_control/app/conf/config_right_sim_r1.ini";
};