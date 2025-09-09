/****************************************************************************
 * UC3 — CartesianPointingComponent (implementation)
 *
 * Variant MINIMA: mantiene tutto com'era, ma carica gli artwork con
 * YARP ResourceFinder (context + --artworks). Nessun altro cambiamento.
 ****************************************************************************/
#include "CartesianPointingComponent.h"

// C++ std
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cmath>

// JSON
#include <nlohmann/json.hpp>
using ordered_json = nlohmann::ordered_json;

// TF2 LinearMath (opzionale)
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// =============================================================================
// Helper: quaternion from the "flat" array returned by get_pose (16 or 18 elems)
// =============================================================================
// Expected layout (row-major): [ r00 r01 r02 tx  r10 r11 r12 ty  r20 r21 r22 tz  0 0 0 1 ]
static bool quatFromFlatPose(const std::vector<double>& flat, Eigen::Quaterniond& out_q)
{
    if (flat.size() != 16 && flat.size() != 18)
        return false;

    const size_t off = (flat.size() == 18) ? 2u : 0u;

    const double r00 = flat[off + 0],  r01 = flat[off + 1],  r02 = flat[off + 2];
    const double r10 = flat[off + 4],  r11 = flat[off + 5],  r12 = flat[off + 6];
    const double r20 = flat[off + 8],  r21 = flat[off + 9],  r22 = flat[off +10];

    Eigen::Matrix3d R;
    R << r00, r01, r02,
         r10, r11, r12,
         r20, r21, r22;

    // Ortonormalizzazione via SVD (robustezza)
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU(), V = svd.matrixV();
    Eigen::Matrix3d R_ortho = U * V.transpose();
    if (R_ortho.determinant() < 0) { U.col(2) *= -1.0; R_ortho = U * V.transpose(); }

    out_q = Eigen::Quaterniond(R_ortho);
    out_q.normalize();
    return true;
}

// =============================================================================
// start() — startup ROS2/TF2, YARP ports, servizi ROS2 + RF SOLO per artworks
// =============================================================================
bool CartesianPointingComponent::start(int argc, char* argv[])
{
    // 1) Initialize ROS2 if needed
    if (!rclcpp::ok()) {
        rclcpp::init(argc, argv);
    }

    // 2) Avvio (se servono) dei controller cartesiani YARP (come prima)
    if (!yarp::os::Network::exists(cartesianPortLeft)) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Starting r1-cartesian-control LEFT...");
        std::string cmd = std::string("r1-cartesian-control --from ") + cartesianControllerIniPathLeft + " > /dev/null 2>&1 &";
        if (std::system(cmd.c_str()) == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error starting r1-cartesian-control LEFT");
            return false;
        }
    }
    if (!yarp::os::Network::exists(cartesianPortRight)) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Starting r1-cartesian-control RIGHT...");
        std::string cmd = std::string("r1-cartesian-control --from ") + cartesianControllerIniPathRight + " > /dev/null 2>&1 &";
        if (std::system(cmd.c_str()) == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error starting r1-cartesian-control RIGHT");
            return false;
        }
    }

    // 3) Attendi la disponibilità dei server RPC (dev-friendly)
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Waiting for port %s...", cartesianPortLeft.c_str());
    while (!yarp::os::Network::exists(cartesianPortLeft)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Waiting for port %s...", cartesianPortRight.c_str());
    while (!yarp::os::Network::exists(cartesianPortRight)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 4) Crea il nodo ROS2 + TF2 persistente
    m_node = rclcpp::Node::make_shared("CartesianPointingComponentNode");
    m_tfBuffer   = std::make_unique<tf2_ros::Buffer>(m_node->get_clock());
    m_tfListener = std::make_unique<tf2_ros::TransformListener>(*m_tfBuffer);

    // 5) Carica gli artwork via ResourceFinder (SOLO questa parte usa RF)
    {
        yarp::os::ResourceFinder rf;
        rf.setDefaultContext(m_rfDefaultContext);
        rf.setDefault("artworks", m_rfDefaultArtworks);
        rf.configure(argc, argv);

        const std::string artworksName = rf.check("artworks",
            yarp::os::Value(m_rfDefaultArtworks)).asString();

        const std::string artworksPath = rf.findFileByName(artworksName);
        if (artworksPath.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                        "Artwork file '%s' non trovato nel context '%s' (YARP_DATA_DIRS). Proseguo senza target.",
                        artworksName.c_str(), rf.getContext().c_str());
        } else {
            m_artworkCoords = loadArtworkCoordinates(artworksPath);
        }
    }

    // 6) Apri e connetti le porte YARP locali verso i controller
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

    // 7) Servizi ROS2
    m_srvPointAt = m_node->create_service<cartesian_pointing_interfaces::srv::PointAt>(
        "/CartesianPointingComponent/PointAt",
        [this](const std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Request> request,
               std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Response> response)
        {
            this->pointTask(request);
            response->is_ok = true;
            response->error_msg = "";
        }
    );

    m_srvIsPointing = m_node->create_service<cartesian_pointing_interfaces::srv::IsPointing>(
        "/CartesianPointingComponent/IsPointing",
        [this](const std::shared_ptr<cartesian_pointing_interfaces::srv::IsPointing::Request> /*req*/,
               std::shared_ptr<cartesian_pointing_interfaces::srv::IsPointing::Response> res)
        {
            std::lock_guard<std::mutex> lk(m_flagMutex);
            res->is_pointing = m_isPointing;
            res->is_ok = true;
        }
    );

    RCLCPP_INFO(m_node->get_logger(), "CartesianPointingComponent READY");
    return true;
}

// =============================================================================
bool CartesianPointingComponent::close()
{
    if (m_cartesianClient.isValid()) {
        m_cartesianClient.close();
    }
    if (m_cartesianPortLeft.isOpen())  m_cartesianPortLeft.close();
    if (m_cartesianPortRight.isOpen()) m_cartesianPortRight.close();
    rclcpp::shutdown();
    return true;
}

void CartesianPointingComponent::spin()
{
    rclcpp::spin(m_node);
}

// =============================================================================
// pointTask() — core logic of pointing (position-only)
// =============================================================================
void CartesianPointingComponent::pointTask(const std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Request> request)
{
    RCLCPP_DEBUG(m_node->get_logger(), "EXECUTE TASK: '%s'", request->target_name.c_str());

    // 1) Lookup artwork by name
    auto it = m_artworkCoords.find(request->target_name);
    if (it == m_artworkCoords.end() || it->second.size() < 3) {
        RCLCPP_WARN(m_node->get_logger(), "No or invalid TARGET '%s' found", request->target_name.c_str());
        return;
    }
    const auto& coords = it->second;

    // 2) Transform map -> base
    Eigen::Vector3d p_base{coords[0], coords[1], coords[2]};
    geometry_msgs::msg::Point mapPt; mapPt.x = coords[0]; mapPt.y = coords[1]; mapPt.z = coords[2];
    geometry_msgs::msg::Point basePt;
    if (transformPointMapToRobot(mapPt, basePt, m_baseFrame, 1.0)) {
        p_base = Eigen::Vector3d(basePt.x, basePt.y, basePt.z);
    } else {
        RCLCPP_WARN(m_node->get_logger(), "TF map->%s failed; using given coords as %s",
                    m_baseFrame.c_str(), m_baseFrame.c_str());
    }

    // 3) Pre-scan dei due bracci
    std::vector<std::pair<std::string,double>> armDistances;
    std::map<std::string, std::vector<double>> cachedPoseValues;
    if (!preScanArticulatedArms(p_base, armDistances, cachedPoseValues)) {
        RCLCPP_ERROR(m_node->get_logger(), "No arms available for pointing");
        return;
    }

    // 4) Preferenza braccio in base alla Y nel frame torso, poi ordina per distanza
    std::string preferredArm;
    if (!chooseArmByTorsoY(p_base, preferredArm)) preferredArm = "LEFT";
    std::stable_sort(armDistances.begin(), armDistances.end(), [&](auto&a, auto&b){
        if (a.first==preferredArm && b.first!=preferredArm) return true;
        if (a.first!=preferredArm && b.first==preferredArm) return false;
        return a.second < b.second;
    });

    // Flag pointing attivo
    {
        std::lock_guard<std::mutex> lk(m_flagMutex);
        m_isPointing = true;
    }

    // 5) Prova con ciascun braccio in ordine di preferenza
    for (const auto &armEntry : armDistances) {
        const std::string armName = armEntry.first;
        yarp::os::Port* activePort = (armName == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
        const std::string remotePort = (armName == "LEFT") ? "/r1-cartesian-control/left_arm/rpc:i"
                                                             : "/r1-cartesian-control/right_arm/rpc:i";
        if (!activePort->isOpen() || !yarp::os::Network::isConnected(activePort->getName(), remotePort)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: port not ready, skipping", armName.c_str());
            continue;
        }

        Eigen::Vector3d shoulder_base;
        if (!getShoulderPosInBase(armName, shoulder_base)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: missing shoulder TF", armName.c_str());
            continue;
        }

        // Candidato sulla retta spalla->target entro raggio
        Eigen::Vector3d candidate = sphereReachPoint(shoulder_base, p_base);

        // Mantieni l'orientazione attuale (fallback identità)
        Eigen::Quaterniond q_keep(1,0,0,0);
        if (auto itPose = cachedPoseValues.find(armName); itPose != cachedPoseValues.end()) {
            Eigen::Quaterniond q_tmp; if (quatFromFlatPose(itPose->second, q_tmp)) q_keep = q_tmp;
        }

        // Verifica raggiungibilità
        if (!isPoseReachable(activePort, candidate, q_keep)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: candidate not reachable, trying next arm", armName.c_str());
            continue;
        }

        // Comando go_to_pose (posizione + orientazione corrente)
        yarp::os::Bottle cmd, res;
        cmd.addString("go_to_pose");
        cmd.addFloat64(candidate.x());
        cmd.addFloat64(candidate.y());
        cmd.addFloat64(candidate.z());
        cmd.addFloat64(q_keep.x());
        cmd.addFloat64(q_keep.y());
        cmd.addFloat64(q_keep.z());
        cmd.addFloat64(q_keep.w());
        cmd.addFloat64(5.0); // durata traiettoria (s)

        bool ok = activePort->write(cmd, res);
        if (ok && res.size()>0 && res.get(0).asVocab32()==yarp::os::createVocab32('o','k')) {
            RCLCPP_DEBUG(m_node->get_logger(), "SUCCESS: %s moved to candidate keeping orientation", armName.c_str());
            break; // fatto con un braccio
        } else {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: go_to_pose failed, trying next arm", armName.c_str());
            continue;
        }
    }

    // Clear flag
    { std::lock_guard<std::mutex> lk(m_flagMutex); m_isPointing = false; }
}

// =============================================================================
// loadArtworkCoordinates() — loads { name : [x,y,z] }
// =============================================================================
std::map<std::string, std::vector<double>>
CartesianPointingComponent::loadArtworkCoordinates(const std::string& filename)
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
            RCLCPP_DEBUG(m_node->get_logger(), "Loaded target '%s' [%.2f %.2f %.2f]", name.c_str(), x,y,z);
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(m_node->get_logger(), "Error parsing '%s': %s", filename.c_str(), ex.what());
    }
    return artworkMap;
}

// =============================================================================
// transformPointMapToRobot() — persistent TF2
// =============================================================================
bool CartesianPointingComponent::transformPointMapToRobot(const geometry_msgs::msg::Point& map_point,
                                                          geometry_msgs::msg::Point& out_robot_point,
                                                          const std::string& robot_frame,
                                                          double timeout_sec)
{
    if (!m_tfBuffer) {
        RCLCPP_WARN(m_node->get_logger(), "TF buffer not initialized");
        return false;
    }
    const std::string map_frame = m_mapFrame;
    const auto timeout = tf2::durationFromSec(timeout_sec);

    if (!m_tfBuffer->canTransform(robot_frame, map_frame, tf2::TimePointZero, timeout)) {
        RCLCPP_WARN(m_node->get_logger(), "TF: transform %s <- %s not available",
                    robot_frame.c_str(), map_frame.c_str());
        return false;
    }

    try {
        auto tf_stamped = m_tfBuffer->lookupTransform(robot_frame, map_frame, tf2::TimePointZero, timeout);
        geometry_msgs::msg::PointStamped p_in, p_out;
        p_in.header = tf_stamped.header;
        p_in.header.frame_id = map_frame;
        p_in.point = map_point;
        tf2::doTransform(p_in, p_out, tf_stamped);
        out_robot_point = p_out.point;
        return true;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(m_node->get_logger(), "TF exception: %s", ex.what());
        return false;
    }
}

// =============================================================================
// preScanArticulatedArms() — read get_pose and compute distances
// =============================================================================
bool CartesianPointingComponent::preScanArticulatedArms(const Eigen::Vector3d& artwork_pos,
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

        yarp::os::Bottle cmd_get, res_get;
        cmd_get.addString("get_pose");
        if (!clientPort->write(cmd_get, res_get)) {
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: get_pose failed for %s", armId.c_str());
            continue;
        }

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

        const size_t pose_offset = (flat_pose.size()==18)?2u:0u;
        const double hand_tx = flat_pose[pose_offset + 3];
        const double hand_ty = flat_pose[pose_offset + 7];
        const double hand_tz = flat_pose[pose_offset + 11];

        const double dist = (Eigen::Vector3d(artwork_pos) - Eigen::Vector3d(hand_tx, hand_ty, hand_tz)).norm();
        armDistances.emplace_back(armId, dist);
        cachedPoseValues.emplace(armId, std::move(flat_pose));
    }
    return !armDistances.empty();
}

// =============================================================================
// isPoseReachable() — RPC wrapper (minima: risponde ok/!ok)
// =============================================================================
bool CartesianPointingComponent::isPoseReachable(yarp::os::Port* activePort,
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
// TF helpers: getTFMatrix(), transformPoint(), chooseArmByTorsoY(), shoulder pos
// =============================================================================
bool CartesianPointingComponent::getTFMatrix(const std::string& target,
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

Eigen::Vector3d CartesianPointingComponent::transformPoint(const Eigen::Matrix4d& T,
                                                           const Eigen::Vector3d& p)
{
    Eigen::Vector4d ph; ph << p, 1.0;
    ph = T*ph;
    return ph.head<3>();
}

bool CartesianPointingComponent::chooseArmByTorsoY(const Eigen::Vector3d& p_base,
                                                   std::string& outArm) const
{
    Eigen::Matrix4d T_torso_base;
    if (!getTFMatrix(m_torsoFrame, m_baseFrame, T_torso_base)) return false;
    Eigen::Vector3d p_torso = transformPoint(T_torso_base, p_base);
    outArm = (p_torso.y() > 0.0) ? "LEFT" : "RIGHT";
    return true;
}

bool CartesianPointingComponent::getShoulderPosInBase(const std::string& arm,
                                                      Eigen::Vector3d& out_pos) const
{
    const std::string& shoulder = (arm=="LEFT") ? m_lShoulderFrame : m_rShoulderFrame;
    Eigen::Matrix4d T_base_sh;
    if (!getTFMatrix(m_baseFrame, shoulder, T_base_sh)) return false;
    out_pos = T_base_sh.block<3,1>(0,3);
    return true;
}

// =============================================================================
// sphereReachPoint() — candidate on the shoulder→target line
// =============================================================================
Eigen::Vector3d CartesianPointingComponent::sphereReachPoint(const Eigen::Vector3d& shoulder_base,
                                                             const Eigen::Vector3d& target_base) const
{
    const double d = (target_base - shoulder_base).norm();
    const double R = std::min(m_reachRadius, std::max(d, m_minDist));
    const double L = std::min(std::max(d - m_safetyBackoff, 0.0), R);
    if (d < 1e-6) return shoulder_base;
    const Eigen::Vector3d dir = (target_base - shoulder_base) / d;
    return shoulder_base + L * dir;
}
```
