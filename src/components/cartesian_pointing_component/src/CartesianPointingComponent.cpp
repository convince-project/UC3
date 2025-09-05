/****************************** CartesianPointingComponent.cpp ******************
 * Implementation
 * - Position-only pointing: keep current EEF orientation
 * - Candidate point on shoulder→target line (reach sphere)
 * - Persistent TF2 buffer/listener
 *******************************************************************************/
#include "CartesianPointingComponent.h"

// C++ std
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cmath>
#include <iostream>

// JSON
#include <nlohmann/json.hpp>
using ordered_json = nlohmann::ordered_json;

// TF2 LinearMath (optional)
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// =============================================================================
// Helper: quaternion from the flattened pose returned by YARP get_pose (16|18)
// =============================================================================
static bool quatFromFlatPose(const std::vector<double>& flat, Eigen::Quaterniond& out_q)
{
    if (flat.size() != 16 && flat.size() != 18) return false;
    const size_t off = (flat.size() == 18) ? 2u : 0u;

    const double r00 = flat[off + 0],  r01 = flat[off + 1],  r02 = flat[off + 2];
    const double r10 = flat[off + 4],  r11 = flat[off + 5],  r12 = flat[off + 6];
    const double r20 = flat[off + 8],  r21 = flat[off + 9],  r22 = flat[off +10];

    Eigen::Matrix3d R;
    R << r00, r01, r02,
         r10, r11, r12,
         r20, r21, r22;

    // Orthonormalize with SVD for numerical robustness
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU(), V = svd.matrixV();
    Eigen::Matrix3d R_ortho = U * V.transpose();
    if (R_ortho.determinant() < 0) { U.col(2) *= -1.0; R_ortho = U * V.transpose(); }

    out_q = Eigen::Quaterniond(R_ortho);
    out_q.normalize();
    return true;
}

// =============================================================================
// start() — ROS2, TF2, YARP, services
// =============================================================================
bool CartesianPointingComponent::start(int argc, char* argv[])
{
    // 1) Initialize ROS2 if needed
    if (!rclcpp::ok()) rclcpp::init(argc, argv);

    // 2) Ensure YARP controllers are up (optional auto-start)
    if (!yarp::os::Network::exists(cartesianPortLeft)) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Starting r1-cartesian-control LEFT...");
        std::string cmd = std::string("r1-cartesian-control --from ")
                          + "/home/user1/ergocub-cartesian-control/src/r1_cartesian_control/app/conf/config_left_sim_r1.ini"
                          + " > /dev/null 2>&1 &";
        if (std::system(cmd.c_str()) == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error starting left controller");
            return false;
        }
    }
    if (!yarp::os::Network::exists(cartesianPortRight)) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Starting r1-cartesian-control RIGHT...");
        std::string cmd = std::string("r1-cartesian-control --from ")
                          + "/home/user1/ergocub-cartesian-control/src/r1_cartesian_control/app/conf/config_right_sim_r1.ini"
                          + " > /dev/null 2>&1 &";
        if (std::system(cmd.c_str()) == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error starting right controller");
            return false;
        }
    }

    // 3) Wait for RPC servers (dev-friendly: no timeout)
    while (!yarp::os::Network::exists(cartesianPortLeft))  std::this_thread::sleep_for(std::chrono::seconds(1));
    while (!yarp::os::Network::exists(cartesianPortRight)) std::this_thread::sleep_for(std::chrono::seconds(1));

    // 4) ROS2 node and TF2
    m_node = rclcpp::Node::make_shared("CartesianPointingComponentNode");
    m_tfBuffer   = std::make_unique<tf2_ros::Buffer>(m_node->get_clock());
    m_tfListener = std::make_unique<tf2_ros::TransformListener>(*m_tfBuffer);

    // 5) Load targets
    m_artworkCoords = loadArtworkCoordinates("/home/user1/UC3/conf/artwork_coords.json");

    // 6) Open local YARP RPC client ports and connect
    if (yarp::os::Network::exists(m_cartesianPortNameLeft))
        yarp::os::Network::disconnect(m_cartesianPortNameLeft, cartesianPortLeft);
    if (!m_cartesianPortLeft.open(m_cartesianPortNameLeft)) {
        yError() << "Cannot open Left RPC client port";
        return false;
    }
    yarp::os::Network::connect(m_cartesianPortNameLeft, cartesianPortLeft);

    if (yarp::os::Network::exists(m_cartesianPortNameRight))
        yarp::os::Network::disconnect(m_cartesianPortNameRight, cartesianPortRight);
    if (!m_cartesianPortRight.open(m_cartesianPortNameRight)) {
        yError() << "Cannot open Right RPC client port";
        return false;
    }
    yarp::os::Network::connect(m_cartesianPortNameRight, cartesianPortRight);

    // 7) ROS2 services
    m_srvPointAt = m_node->create_service<cartesian_pointing_interfaces::srv::PointAt>(
        "/CartesianPointingComponent/PointAt",
        [this](const std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Request> req,
               std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Response> res)
        {
            // Run the pointing task in this thread (or spawn if you prefer)
            this->pointTask(req);
            res->is_ok = true;
            res->error_msg = "";
        });

    m_srvIsPointing = m_node->create_service<cartesian_pointing_interfaces::srv::IsPointing>(
        "/CartesianPointingComponent/IsPointing",
        [this](const std::shared_ptr<cartesian_pointing_interfaces::srv::IsPointing::Request> /*req*/,
               std::shared_ptr<cartesian_pointing_interfaces::srv::IsPointing::Response> res)
        {
            std::lock_guard<std::mutex> lk(m_flagMutex);
            res->is_pointing = m_isPointing;
            res->is_ok = true;
        });

    RCLCPP_INFO(m_node->get_logger(), "CartesianPointingComponent READY");
    return true;
}

// =============================================================================
bool CartesianPointingComponent::close()
{
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
// pointTask() — core pointing logic (position-only) + wait for motion
// =============================================================================
void CartesianPointingComponent::pointTask(
    const std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Request> request)
{
    RCLCPP_INFO(m_node->get_logger(), "PointAt request: '%s'", request->target_name.c_str());

    // 1) Lookup target in map
    auto it = m_artworkCoords.find(request->target_name);
    if (it == m_artworkCoords.end() || it->second.size() < 3) {
        RCLCPP_WARN(m_node->get_logger(), "Target '%s' not found or invalid.", request->target_name.c_str());
        return;
    }
    const auto& coords = it->second;

    // 2) Transform to base
    Eigen::Vector3d p_base{coords[0], coords[1], coords[2]};
    geometry_msgs::msg::Point mapPt; mapPt.x = coords[0]; mapPt.y = coords[1]; mapPt.z = coords[2];
    geometry_msgs::msg::Point basePt;
    if (transformPointMapToRobot(mapPt, basePt, m_baseFrame, 1.0)) {
        p_base = Eigen::Vector3d(basePt.x, basePt.y, basePt.z);
    } else {
        RCLCPP_WARN(m_node->get_logger(), "TF map->%s failed; using given coords as %s",
                    m_baseFrame.c_str(), m_baseFrame.c_str());
    }

    // 3) Pre-scan arms
    std::vector<std::pair<std::string,double>> armDistances;
    std::map<std::string, std::vector<double>> cachedPoseValues;
    if (!preScanArticulatedArms(p_base, armDistances, cachedPoseValues)) {
        RCLCPP_ERROR(m_node->get_logger(), "No arms available.");
        return;
    }

    // 4) Prefer arm by torso Y (LEFT if y>0), then sort by distance
    std::string preferredArm;
    if (!chooseArmByTorsoY(p_base, preferredArm)) preferredArm = "LEFT";
    std::stable_sort(armDistances.begin(), armDistances.end(),
                     [&](auto&a, auto&b){
                         if (a.first==preferredArm && b.first!=preferredArm) return true;
                         if (a.first!=preferredArm && b.first==preferredArm) return false;
                         return a.second < b.second;
                     });

    // Set pointing flag (cleared on exit)
    {
        std::lock_guard<std::mutex> lk(m_flagMutex);
        m_isPointing = true;
    }

    // 5) Try each arm
    for (const auto &armEntry : armDistances) {
        const std::string armName = armEntry.first;
        yarp::os::Port* activePort = (armName == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
        const std::string remotePort = (armName == "LEFT")
            ? "/r1-cartesian-control/left_arm/rpc:i"
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

        // Candidate on shoulder→target line
        Eigen::Vector3d candidate = sphereReachPoint(shoulder_base, p_base);

        // Keep current orientation (fallback identity)
        Eigen::Quaterniond q_keep(1,0,0,0);
        if (auto itPose = cachedPoseValues.find(armName); itPose != cachedPoseValues.end()) {
            Eigen::Quaterniond q_tmp; if (quatFromFlatPose(itPose->second, q_tmp)) q_keep = q_tmp;
        }

        // Reachability check
        if (!isPoseReachable(activePort, candidate, q_keep)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: candidate not reachable, trying next arm", armName.c_str());
            continue;
        }

        // Command go_to_pose
        constexpr double kGoToPoseDurationSec = 5.0;
        yarp::os::Bottle cmd, res;
        cmd.addString("go_to_pose");
        cmd.addFloat64(candidate.x());
        cmd.addFloat64(candidate.y());
        cmd.addFloat64(candidate.z());
        cmd.addFloat64(q_keep.x());
        cmd.addFloat64(q_keep.y());
        cmd.addFloat64(q_keep.z());
        cmd.addFloat64(q_keep.w());
        cmd.addFloat64(kGoToPoseDurationSec);

        bool ok = activePort->write(cmd, res);
        const bool accepted = ok && res.size()>0 &&
          (!res.get(0).isVocab32() || res.get(0).asVocab32()==yarp::os::createVocab32('o','k'));
        if (!accepted) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: go_to_pose refused/failed: %s",
                        armName.c_str(), res.toString().c_str());
            continue;
        }

        // Wait for completion via is_motion_done()
        using namespace std::chrono_literals;
        const auto t0 = std::chrono::steady_clock::now();
        const auto kMaxWait = std::chrono::milliseconds(static_cast<int>(kGoToPoseDurationSec * 1800)); // ~1.8×
        std::this_thread::sleep_for(300ms);
        while (true) {
            yarp::os::Bottle cmd_done, res_done;
            cmd_done.addString("is_motion_done");
            bool ok_done = activePort->write(cmd_done, res_done);

            bool finished = false;
            if (ok_done && res_done.size()>0) {
                if (res_done.get(0).isVocab32() &&
                    res_done.get(0).asVocab32()==yarp::os::createVocab32('o','k')) {
                    if (res_done.size()>=2 && res_done.get(1).isBool())   finished = res_done.get(1).asBool();
                    else if (res_done.size()>=2 && res_done.get(1).isInt32()) finished = res_done.get(1).asInt32()!=0;
                } else if (res_done.get(0).isBool()) {
                    finished = res_done.get(0).asBool();
                } else if (res_done.get(0).isInt32()) {
                    finished = res_done.get(0).asInt32()!=0;
                }
            }

            if (finished) {
                RCLCPP_INFO(m_node->get_logger(), "ARM %s: motion completed.", armName.c_str());
                break;
            }
            if (std::chrono::steady_clock::now() - t0 > kMaxWait) {
                RCLCPP_WARN(m_node->get_logger(), "ARM %s: timeout waiting is_motion_done().", armName.c_str());
                break;
            }
            std::this_thread::sleep_for(150ms);
        }

        // Optionally go_home (uncomment if desired)
        /*
        {
            yarp::os::Bottle cmd_home, res_home;
            cmd_home.addString("go_home");
            bool home_ok = activePort->write(cmd_home, res_home);
            if (!home_ok || res_home.size()==0 ||
                (res_home.get(0).isVocab32() &&
                 res_home.get(0).asVocab32()!=yarp::os::createVocab32('o','k'))) {
                RCLCPP_ERROR(m_node->get_logger(), "ARM %s: go_home failed: %s",
                             armName.c_str(), res_home.toString().c_str());
            } else {
                RCLCPP_INFO(m_node->get_logger(), "ARM %s: go_home accepted.", armName.c_str());
            }
        }
        */

        // Done with one arm
        break;
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
// isPoseReachable() — RPC wrapper
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
    if (!ok || res.size()==0) return false;

    if (res.get(0).isVocab32() && res.get(0).asVocab32()==yarp::os::createVocab32('o','k')) {
        if (res.size()>=2 && res.get(1).isBool())   return res.get(1).asBool();
        if (res.size()>=2 && res.get(1).isInt32())  return res.get(1).asInt32()!=0;
    }
    if (res.get(0).isBool())  return res.get(0).asBool();
    if (res.get(0).isInt32()) return res.get(0).asInt32()!=0;
    return false;
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
