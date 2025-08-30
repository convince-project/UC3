/******************************************************************************
 *                                                                            *
 * UC3 — ExecuteDanceComponent (implementation)                                *
 *                                                                            *
 * Nota: include anche la fix per lo yaw (Matrix3x3.getRPY)                    *
 *                                                                            *
 ******************************************************************************/
#include "ExecuteDanceComponent.h"

// ==== C++ std ====
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cmath>

// ==== JSON (nlohmann) per caricare le coordinate delle opere ====
#include <nlohmann/json.hpp>
using ordered_json = nlohmann::ordered_json;

// ==== TF2 LinearMath per la conversione Quaternion -> RPY (fix getYaw) ====
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// =============================================================================
// start() — avvio di ROS2, TF2, porte YARP e servizio ROS2
// =============================================================================
bool ExecuteDanceComponent::start(int argc, char* argv[])
{
    // 1) Inizializza ROS2 se necessario
    if (!rclcpp::ok()) {
        rclcpp::init(argc, argv);
    }

    // 2) Avvia i controller 
    if (!yarp::os::Network::exists(cartesianPortLeft)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Avvio r1-cartesian-control LEFT...");
        std::string cmd = std::string("r1-cartesian-control --from ") + cartesianControllerIniPathLeft + " > /dev/null 2>&1 &";
        if (std::system(cmd.c_str()) == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Errore nell'avvio di r1-cartesian-control LEFT");
            return false;
        }
    }
    if (!yarp::os::Network::exists(cartesianPortRight)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Avvio r1-cartesian-control RIGHT...");
        std::string cmd = std::string("r1-cartesian-control --from ") + cartesianControllerIniPathRight + " > /dev/null 2>&1 &";
        if (std::system(cmd.c_str()) == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Errore nell'avvio di r1-cartesian-control RIGHT");
            return false;
        }
    }

    // 3) Attendi che i server RPC compaiano (no timeout: comodo in dev)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attesa della porta %s...", cartesianPortLeft.c_str());
    while (!yarp::os::Network::exists(cartesianPortLeft)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attesa della porta %s...", cartesianPortRight.c_str());
    while (!yarp::os::Network::exists(cartesianPortRight)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 4) Crea il nodo ROS2 e il TF2 persistente (buffer+listener)
    m_node = rclcpp::Node::make_shared("ExecuteDanceComponentNode");
    m_tfBuffer   = std::make_unique<tf2_ros::Buffer>(m_node->get_clock());
    m_tfListener = std::make_unique<tf2_ros::TransformListener>(*m_tfBuffer);

    // 5) Carica le coordinate delle opere (in frame "map")
    m_artworkCoords = loadArtworkCoordinates("/home/user1/UC3/conf/artwork_coords.json");

    // 6) Apri e connetti le porte RPC locali verso i due controller cartesiani
    if (yarp::os::Network::exists(m_cartesianPortNameLeft)) {
        yarp::os::Network::disconnect(m_cartesianPortNameLeft, cartesianPortLeft);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    if (!m_cartesianPortLeft.open(m_cartesianPortNameLeft)) {
        yError() << "Cannot open Left CartesianController client port";
        return false;
    }
    yarp::os::Network::connect(m_cartesianPortNameLeft, cartesianPortLeft);

    if (yarp::os::Network::exists(m_cartesianPortNameRight)) {
        yarp::os::Network::disconnect(m_cartesianPortNameRight, cartesianPortRight);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    if (!m_cartesianPortRight.open(m_cartesianPortNameRight)) {
        yError() << "Cannot open Right CartesianController client port";
        return false;
    }
    yarp::os::Network::connect(m_cartesianPortNameRight, cartesianPortRight);

    // 7) Esponi il servizio ROS2 per avviare il pointing verso un'opera (per nome)
    m_executeDanceService = m_node->create_service<execute_dance_interfaces::srv::ExecuteDance>(
        "/ExecuteDanceComponent/ExecuteDance",
        [this](const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request,
               std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Response> response)
        {
            this->executeTask(request); 
            response->is_ok = true;
            response->error_msg = "";
        }
    );

    return true;
}

// =============================================================================
// close() — chiude porte e spegne ROS2
// =============================================================================
bool ExecuteDanceComponent::close()
{
    if (m_cartesianClient.isValid()) {
        m_cartesianClient.close();
    }
    if (m_cartesianPortLeft.isOpen())  m_cartesianPortLeft.close();
    if (m_cartesianPortRight.isOpen()) m_cartesianPortRight.close();
    rclcpp::shutdown();
    return true;
}

void ExecuteDanceComponent::spin()
{
    rclcpp::spin(m_node);
}

// =============================================================================
// executeTask() — cuore della logica di pointing
// =============================================================================
void ExecuteDanceComponent::executeTask(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request)
{
    RCLCPP_INFO(m_node->get_logger(), "EXECUTE TASK: '%s'", request->dance_name.c_str());

    // 1) Recupera le coordinate dell'opera (in frame map)
    auto it = m_artworkCoords.find(request->dance_name);
    if (it == m_artworkCoords.end() || it->second.size() < 3) {
        RCLCPP_WARN(m_node->get_logger(), "No or invalid ARTWORK '%s' found", request->dance_name.c_str());
        return;
    }
    const auto& coords = it->second;

    // 2) Trasforma il punto da map → base_link (se TF disponibile); fallback: usa map come base
    Eigen::Vector3d p_base{coords[0], coords[1], coords[2]};
    geometry_msgs::msg::Point mapPt;
    mapPt.x = coords[0]; mapPt.y = coords[1]; mapPt.z = coords[2];

    geometry_msgs::msg::Point basePt;
    if (transformPointMapToRobot(mapPt, basePt, m_baseFrame, 1.0)) {
        p_base = Eigen::Vector3d(basePt.x, basePt.y, basePt.z);
    } else {
        RCLCPP_WARN(m_node->get_logger(), "TF map->%s failed; assuming coords already in base", m_baseFrame.c_str());
    }

    // 3) Pre-scan: interroga i due controller per la posa mano e stima distanza dal target
    std::vector<std::pair<std::string,double>> armDistances;                 // (arm, distanza)
    std::map<std::string, std::vector<double>> cachedPoseValues;             // (arm -> flat pose)
    if (!preScanArticulatedArms(p_base, armDistances, cachedPoseValues)) {
        RCLCPP_ERROR(m_node->get_logger(), "No arms available for pointing");
        return;
    }

    // 4) Scegli braccio in stile thread camera: segno della Y in torso
    std::string preferredArm;
    if (!chooseArmByTorsoY(p_base, preferredArm)) preferredArm = "LEFT";    // fallback
    std::stable_sort(armDistances.begin(), armDistances.end(), [&](auto&a, auto&b){
        if (a.first==preferredArm && b.first!=preferredArm) return true;      // preferito prima
        if (a.first!=preferredArm && b.first==preferredArm) return false;
        return a.second < b.second;                                           // poi per distanza
    });

    // 5) Prova i bracci nell'ordine stabilito
    for (const auto &armEntry : armDistances) {
        const std::string armName = armEntry.first;                           // "LEFT" o "RIGHT"
        yarp::os::Port* activePort = (armName == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
        const std::string remotePort = (armName == "LEFT") ? "/r1-cartesian-control/left_arm/rpc:i"
                                                             : "/r1-cartesian-control/right_arm/rpc:i";
        if (!activePort->isOpen() || !yarp::os::Network::isConnected(activePort->getName(), remotePort)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: port not ready, skipping", armName.c_str());
            continue;
        }

        // Recupera la flat pose cache del braccio
        auto it_cached = cachedPoseValues.find(armName);
        if (it_cached == cachedPoseValues.end()) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: no cached pose", armName.c_str());
            continue;
        }

        // 5.a) Orientazione: usa la direzione spalla→target in BASE e allinea l'asse strumentale
        Eigen::Vector3d shoulder_base;
        if (!getShoulderPosInBase(armName, shoulder_base)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: missing shoulder TF", armName.c_str());
            continue;
        }
        Eigen::Vector3d dir_base = p_base - shoulder_base;                    // direzione di pointing
        if (dir_base.norm() < 1e-6) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: degenerate target near shoulder", armName.c_str());
            continue;
        }
        Eigen::Quaterniond q_target = quatAlignAxisToDir(dir_base, m_toolAxis, Eigen::Vector3d::UnitY());

        // 5.b) Info mano corrente (per fallback binary search)
        const std::vector<double>& flat = it_cached->second;
        const size_t pose_offset = (flat.size()==18)?2u:0u;                   // alcune risposte hanno header
        Eigen::Vector3d hand_pos(flat[pose_offset+3], flat[pose_offset+7], flat[pose_offset+11]);
        Eigen::Vector3d vec_to_target = (p_base - hand_pos);
        double vec_norm = vec_to_target.norm();
        if (vec_norm < 1e-9) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: hand already at target", armName.c_str());
            continue;
        }
        vec_to_target /= vec_norm;

        // 5.c) Controllo reachability sul target esatto con q_target
        if (!isPoseReachable(activePort, p_base, q_target)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: exact target not reachable", armName.c_str());
            continue;                                                          // prova l'altro braccio
        }

        // 5.d) Primo candidato "furbo": sulla sfera spalla→target (stile thread camera)
        Eigen::Vector3d first_candidate = sphereReachPoint(shoulder_base, p_base);
        Eigen::Vector3d best_candidate  = first_candidate;
        if (!isPoseReachable(activePort, first_candidate, q_target)) {
            // Fallback: binary search lungo mano→target (tua logica originale)
            if (!probeBinarySearch(activePort, hand_pos, p_base, vec_to_target, vec_norm, q_target, best_candidate)) {
                RCLCPP_WARN(m_node->get_logger(), "ARM %s: probing failed", armName.c_str());
                continue;
            }
        }

        // 5.e) Invia il movimento finale al controller cartesiano
        yarp::os::Bottle cmd_pose_final, res_pose_final;
        cmd_pose_final.addString("go_to_pose");
        cmd_pose_final.addFloat64(best_candidate.x());
        cmd_pose_final.addFloat64(best_candidate.y());
        cmd_pose_final.addFloat64(best_candidate.z());
        cmd_pose_final.addFloat64(q_target.x());
        cmd_pose_final.addFloat64(q_target.y());
        cmd_pose_final.addFloat64(q_target.z());
        cmd_pose_final.addFloat64(q_target.w());
        cmd_pose_final.addFloat64(15.0);                                        // durata traiettoria (tarabile)

        bool ok = activePort->write(cmd_pose_final, res_pose_final);
        if (ok && res_pose_final.size()>0 && res_pose_final.get(0).asVocab32()==yarp::os::createVocab32('o','k')) {
            RCLCPP_INFO(m_node->get_logger(), "SUCCESS: %s aligned toward '%s'", armName.c_str(), request->dance_name.c_str());
            break; // missione compiuta
        } else {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: go_to_pose failed, trying next arm", armName.c_str());
            continue;
        }
    }
}

// // =============================================================================
// // amclPoseCallback() — esempio di aggiornamento stato (usa getRPY, non getYaw)
// // =============================================================================
// void ExecuteDanceComponent::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
// {
//     const auto &p = msg->pose.pose.position;
//     const auto &q = msg->pose.pose.orientation;
//     m_currentX = p.x;
//     m_currentY = p.y;

//     // Quaternion ROS → tf2::Quaternion
//     tf2::Quaternion qq(q.x, q.y, q.z, q.w);

//     // Ottieni roll,pitch,yaw in modo portabile (fix per getYaw non disponibile)
//     double roll=0.0, pitch=0.0, yaw=0.0;
//     tf2::Matrix3x3(qq).getRPY(roll, pitch, yaw);
//     m_currentYaw = yaw;
// }

// =============================================================================
// Reachability helpers & wrappers
// =============================================================================
bool ExecuteDanceComponent::checkPoseReachabilityForArm(double x, double y, double z, const std::string& armName)
{
    yarp::os::Port* activePort = (armName == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
    yarp::os::Bottle cmd, res;
    cmd.addString("is_pose_reachable");
    cmd.addFloat64(x); cmd.addFloat64(y); cmd.addFloat64(z);
    // orientamento neutro (se serve un pre-check grossolano)
    cmd.addFloat64(0.0); cmd.addFloat64(0.0); cmd.addFloat64(0.0); cmd.addFloat64(1.0);
    bool ok = activePort->write(cmd, res);
    return ok && res.size()>0 && res.get(0).asVocab32()==yarp::os::createVocab32('o','k');
}

// bool ExecuteDanceComponent::sendPositionCommand(double x, double y, double z, const std::string& armName)
// {
//     yarp::os::Port* activePort = (armName == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
//     yarp::os::Bottle cmd, res;
//     cmd.addString("go_to_position");
//     cmd.addFloat64(x); cmd.addFloat64(y); cmd.addFloat64(z);
//     cmd.addFloat64(30.0);
//     bool ok = activePort->write(cmd, res);
//     return ok && res.size()>0 && res.get(0).asVocab32()==yarp::os::createVocab32('o','k');
// }

bool ExecuteDanceComponent::checkPoseReachability(double x, double y, double z)
{
    return checkPoseReachabilityForArm(x,y,z,"LEFT");
}

// =============================================================================
// loadArtworkCoordinates() — carica { "artworks": { name: {x,y,z} } }
// =============================================================================
std::map<std::string, std::vector<double>> ExecuteDanceComponent::loadArtworkCoordinates(const std::string& filename)
{
    std::map<std::string, std::vector<double>> artworkMap;
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(m_node->get_logger(), "Unable to open artwork file '%s'", filename.c_str());
            return artworkMap;
        }
        auto config = ordered_json::parse(file);
        for (auto& [name, data] : config.at("artworks").items()) {
            double x = data.at("x").get<double>();
            double y = data.at("y").get<double>();
            double z = data.at("z").get<double>();
            artworkMap.emplace(name, std::vector<double>{x,y,z});
            RCLCPP_INFO(m_node->get_logger(), "Loaded artwork '%s' [%.2f %.2f %.2f]", name.c_str(), x,y,z);
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(m_node->get_logger(), "Error parsing '%s': %s", filename.c_str(), ex.what());
    }
    return artworkMap;
}

// =============================================================================
// transformPointMapToRobot() — helper legacy che usa un listener locale
// =============================================================================
bool ExecuteDanceComponent::transformPointMapToRobot(const geometry_msgs::msg::Point& map_point,
                                                     geometry_msgs::msg::Point& out_robot_point,
                                                     const std::string& robot_frame,
                                                     double timeout_sec)
{
    tf2_ros::Buffer tfBuffer(m_node->get_clock());                // buffer temporaneo
    tf2_ros::TransformListener tfListener(tfBuffer);               // listener temporaneo

    const std::string map_frame = "map";
    const auto timeout_ms = std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000.0));

    if (!tfBuffer.canTransform(robot_frame, map_frame, rclcpp::Time(0), timeout_ms)) {
        RCLCPP_WARN(m_node->get_logger(), "TF: transform %s <- %s not available", robot_frame.c_str(), map_frame.c_str());
        return false;
    }
    try {
        auto tf_stamped = tfBuffer.lookupTransform(robot_frame, map_frame, rclcpp::Time(0));
        geometry_msgs::msg::PointStamped p_in, p_out;
        p_in.header.stamp = tf_stamped.header.stamp;
        p_in.header.frame_id = map_frame;
        p_in.point = map_point;
        tf2::doTransform(p_in, p_out, tf_stamped);                 // applica la trasformazione
        out_robot_point = p_out.point;
        return true;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(m_node->get_logger(), "TF exception: %s", ex.what());
        return false;
    }
}

// =============================================================================
// preScanArticulatedArms() — interroga get_pose e stima distanze dal target
// =============================================================================
bool ExecuteDanceComponent::preScanArticulatedArms(const Eigen::Vector3d& artwork_pos,
                                                   std::vector<std::pair<std::string,double>>& armDistances,
                                                   std::map<std::string, std::vector<double>>& cachedPoseValues)
{
    armDistances.clear();
    cachedPoseValues.clear();

    for (const std::string armId : {"LEFT", "RIGHT"}) {
        yarp::os::Port* clientPort = (armId == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
        const std::string serverPort = (armId == "LEFT") ? "/r1-cartesian-control/left_arm/rpc:i"
                                                           : "/r1-cartesian-control/right_arm/rpc:i";
        if (!clientPort->isOpen() || !yarp::os::Network::isConnected(clientPort->getName(), serverPort)) {
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: %s port not ready", armId.c_str());
            continue;
        }

        // Chiedi la posa (matrice 4x4 appiattita) al controller
        yarp::os::Bottle cmd_get, res_get; cmd_get.addString("get_pose");
        if (!clientPort->write(cmd_get, res_get)) {
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: get_pose failed for %s", armId.c_str());
            continue;
        }

        // Unroll della Bottle in un vettore double flat (16 o 18 elementi)
        std::vector<double> flat_pose;
        for (size_t i = 0; i < res_get.size(); ++i) {
            if (res_get.get(i).isList()) {
                yarp::os::Bottle* sub = res_get.get(i).asList();
                for (size_t j = 0; j < sub->size(); ++j) flat_pose.push_back(sub->get(j).asFloat64());
            } else {
                flat_pose.push_back(res_get.get(i).asFloat64());
            }
        }
        if (flat_pose.size() != 18 && flat_pose.size() != 16) {
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: unexpected pose size=%zu for %s", flat_pose.size(), armId.c_str());
            continue;
        }

        // Estrai la traslazione della mano (elementi 3,7,11 con eventuale offset 2)
        const size_t pose_offset = (flat_pose.size()==18)?2u:0u;
        const double hand_tx = flat_pose[pose_offset + 3];
        const double hand_ty = flat_pose[pose_offset + 7];
        const double hand_tz = flat_pose[pose_offset + 11];

        // Distanza mano→target (solo per ordinare preferenze)
        const double dist = (Eigen::Vector3d(artwork_pos) - Eigen::Vector3d(hand_tx, hand_ty, hand_tz)).norm();
        armDistances.emplace_back(armId, dist);
        cachedPoseValues.emplace(armId, std::move(flat_pose));
    }
    return !armDistances.empty();
}

// // =============================================================================
// // computeOrientationFromFlatPose() — metodo legacy (non usato di default)
// // =============================================================================
// bool ExecuteDanceComponent::computeOrientationFromFlatPose(const std::vector<double>& flat_pose,
//                                                            const Eigen::Vector3d& artwork_pos,
//                                                            Eigen::Vector3d& hand_pos,
//                                                            Eigen::Vector3d& vec_to_artwork,
//                                                            double& vec_norm,
//                                                            Eigen::Matrix3d& R_hand,
//                                                            Eigen::Quaterniond& q_target)
// {
//     const size_t pose_offset = (flat_pose.size() == 18) ? 2u : 0u;
//     if (flat_pose.size() < pose_offset + 12) return false;

//     hand_pos = Eigen::Vector3d(flat_pose[pose_offset+3], flat_pose[pose_offset+7], flat_pose[pose_offset+11]);
//     Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::RowMajor>> T_row(flat_pose.data() + pose_offset);
//     R_hand = T_row.topLeftCorner<3,3>();

//     vec_to_artwork = artwork_pos - hand_pos;
//     vec_norm = vec_to_artwork.norm();
//     if (vec_norm <= 1e-9) return false;
//     vec_to_artwork.normalize();

//     const Eigen::Vector3d hand_local_x = R_hand.col(0);
//     const double cos_angle = std::clamp(hand_local_x.dot(vec_to_artwork), -1.0, 1.0);
//     Eigen::Vector3d rotation_axis = hand_local_x.cross(vec_to_artwork);
//     double axis_norm = rotation_axis.norm();

//     if (axis_norm <= 1e-6) {
//         if (cos_angle > 0.999999) {
//             q_target = Eigen::Quaterniond(R_hand);
//         } else {
//             Eigen::Vector3d fallback = (std::abs(hand_local_x.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
//             rotation_axis = hand_local_x.cross(fallback);
//             if (rotation_axis.norm() <= 1e-9) rotation_axis = Eigen::Vector3d::UnitZ();
//             rotation_axis.normalize();
//             Eigen::AngleAxisd aa(M_PI, rotation_axis);
//             q_target = Eigen::Quaterniond(aa * R_hand);
//         }
//     } else {
//         rotation_axis.normalize();
//         double angle = std::acos(cos_angle);
//         q_target = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_axis) * R_hand);
//     }
//     return true;
// }

// =============================================================================
// isPoseReachable() — wrapper RPC
// =============================================================================
bool ExecuteDanceComponent::isPoseReachable(yarp::os::Port* activePort,
                                            const Eigen::Vector3d& candidate,
                                            const Eigen::Quaterniond& q_target)
{
    yarp::os::Bottle cmd, res;
    cmd.addString("is_pose_reachable");
    cmd.addFloat64(candidate.x());
    cmd.addFloat64(candidate.y());
    cmd.addFloat64(candidate.z());
    cmd.addFloat64(q_target.x());
    cmd.addFloat64(q_target.y());
    cmd.addFloat64(q_target.z());
    cmd.addFloat64(q_target.w());
    bool ok = activePort->write(cmd, res);
    return ok && res.size()>0 && res.get(0).asVocab32()==yarp::os::createVocab32('o','k');
}

// =============================================================================
// probeBinarySearch() — ricerca binaria lungo mano→target
// =============================================================================
bool ExecuteDanceComponent::probeBinarySearch(yarp::os::Port* activePort,
                                              const Eigen::Vector3d& hand_pos,
                                              const Eigen::Vector3d& artwork_pos,
                                              const Eigen::Vector3d& vec_to_artwork,
                                              double vec_norm,
                                              const Eigen::Quaterniond& q_target,
                                              Eigen::Vector3d& out_best_candidate)
{
    (void)artwork_pos; // non usato; la direzione è già in vec_to_artwork

    const double safety_margin = 0.05;  // non arrivare fino al target
    const double pos_tol = 1e-3;        // tolleranza su t
    const int    max_iters = 20;        // iterazioni massime

    double max_t = std::max(0.0, vec_norm - safety_margin);
    Eigen::Vector3d candidate_max = hand_pos + vec_to_artwork * max_t;
    if (isPoseReachable(activePort, candidate_max, q_target)) { out_best_candidate = candidate_max; return true; }

    double lo=0.0, hi=max_t, best_t=0.0;
    for (int it = 0; it < max_iters && (hi-lo)>pos_tol; ++it) {
        double mid = 0.5*(lo+hi);
        Eigen::Vector3d c = hand_pos + vec_to_artwork * mid;
        if (isPoseReachable(activePort, c, q_target)) { best_t = mid; lo = mid; } else { hi = mid; }
    }
    out_best_candidate = hand_pos + vec_to_artwork * best_t;
    return true;
}

// =============================================================================
// getTFMatrix(), transformPoint(), chooseArmByTorsoY(), getShoulderPosInBase(),
// quatAlignAxisToDir(), sphereReachPoint() — helpers geometrici/TF
// =============================================================================
bool ExecuteDanceComponent::getTFMatrix(const std::string& target,
                                        const std::string& source,
                                        Eigen::Matrix4d& T) const
{
    if (!m_tfBuffer) return false;
    try {
        auto tf = m_tfBuffer->lookupTransform(target, source, rclcpp::Time(0), std::chrono::seconds(1));
        const auto& tr = tf.transform.translation;
        const auto& q  = tf.transform.rotation;
        Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);
        Eigen::Matrix3d R = Q.toRotationMatrix();
        T.setIdentity();
        T.topLeftCorner<3,3>() = R;
        T(0,3)=tr.x; T(1,3)=tr.y; T(2,3)=tr.z;
        return true;
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN(m_node->get_logger(), "TF lookup failed %s<- %s : %s", target.c_str(), source.c_str(), e.what());
        return false;
    }
}

Eigen::Vector3d ExecuteDanceComponent::transformPoint(const Eigen::Matrix4d& T,
                                                      const Eigen::Vector3d& p)
{
    Eigen::Vector4d ph; ph << p, 1.0; ph = T*ph; return ph.head<3>();
}

bool ExecuteDanceComponent::chooseArmByTorsoY(const Eigen::Vector3d& p_base,
                                              std::string& outArm) const
{
    Eigen::Matrix4d T_torso_base; // torso <- base
    if (!getTFMatrix(m_torsoFrame, m_baseFrame, T_torso_base)) return false;
    Eigen::Vector3d p_torso = transformPoint(T_torso_base, p_base); // coord target espresse in torso
    outArm = (p_torso.y() > 0.0) ? "LEFT" : "RIGHT";             // Y>0: sinistra, altrimenti destra
    return true;
}

bool ExecuteDanceComponent::getShoulderPosInBase(const std::string& arm,
                                                 Eigen::Vector3d& out_pos) const
{
    const std::string& shoulder = (arm=="LEFT") ? m_lShoulderFrame : m_rShoulderFrame;
    Eigen::Matrix4d T_base_sh; // base <- shoulder
    if (!getTFMatrix(m_baseFrame, shoulder, T_base_sh)) return false;
    out_pos = T_base_sh.block<3,1>(0,3); // componente di traslazione
    return true;
}

Eigen::Quaterniond ExecuteDanceComponent::quatAlignAxisToDir(const Eigen::Vector3d& dir_base,
                                                             ToolAxis axis,
                                                             const Eigen::Vector3d& worldUp) const
{
    // Direzione "forward" normalizzata
    Eigen::Vector3d fwd = dir_base.normalized();
    if (axis == ToolAxis::AlignZ) fwd = -fwd; // convenzione: Z dell'EEF punta verso il target (come thread camera)

    // Costruisci un frame ortonormale robusto via Gram-Schmidt
    Eigen::Vector3d up = worldUp;
    if (std::abs(fwd.dot(up)) > 0.999) up = Eigen::Vector3d::UnitX(); // evita quasi parallelismi
    Eigen::Vector3d right = up.cross(fwd).normalized();
    up = fwd.cross(right).normalized();

    // Allestisci la matrice R con l'asse desiderato verso il target
    Eigen::Matrix3d R;
    if (axis == ToolAxis::AlignX) { // X=fwd, Y=up, Z=right
        R.col(0)=fwd; R.col(1)=up; R.col(2)=right;
    } else {                        // Z=fwd, X=right, Y=up
        R.col(0)=right; R.col(1)=up; R.col(2)=fwd;
    }
    return Eigen::Quaterniond(R);
}

Eigen::Vector3d ExecuteDanceComponent::sphereReachPoint(const Eigen::Vector3d& shoulder_base,
                                                        const Eigen::Vector3d& target_base) const
{
    // Punto sulla retta spalla→target ad una distanza L (clippata) dalla spalla
    const double d = (target_base - shoulder_base).norm();
    const double R = std::min(m_reachRadius, std::max(d, m_minDist)); // clamp del raggio
    const double L = std::min(std::max(d - m_safetyBackoff, 0.0), R); // arretra un po' dal target
    if (d < 1e-6) return shoulder_base;
    const Eigen::Vector3d dir = (target_base - shoulder_base) / d;
    return shoulder_base + L * dir;
}