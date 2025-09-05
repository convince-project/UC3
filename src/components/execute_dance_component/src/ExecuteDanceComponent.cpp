/****************************************************************************
 *                                                                          *
 * UC3 — ExecuteDanceComponent (implementation)                              *
 *                                                                          *
 * Variant: position-only (keeps current EEF orientation)                   *
 *  - Candidate point on shoulder→target line                                *
 *  - persistent TF2 (buffer + listener)                                     *
 *                                                                          *
 ****************************************************************************/
#include "ExecuteDanceComponent.h"

// ==== C++ std ====
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cmath>

// ==== JSON (nlohmann) to load artwork coordinates ====
#include <nlohmann/json.hpp>
using ordered_json = nlohmann::ordered_json;

// ==== TF2 LinearMath for Quaternion -> RPY conversion (fix getYaw) ====
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>




// =============================================================================
// Helper: quaternion from the "flat" array returned by get_pose (16 or 18 elems)
// =============================================================================
// Expected layout (row-major): [ r00 r01 r02 tx  r10 r11 r12 ty  r20 r21 r22 tz  0 0 0 1 ]
// Utility: derive a quaternion from the flattened 4x4 matrix returned by the controller
static bool quatFromFlatPose(const std::vector<double>& flat, Eigen::Quaterniond& out_q)
{
    // Check vector has 16 or 18 values
    // (some YARP versions add 2 header values → 18 elements)
    if (flat.size() != 16 && flat.size() != 18) 
        return false;

    // If size is 18, skip the first 2 values (header)
    const size_t off = (flat.size() == 18) ? 2u : 0u;

    // Extract the 9 rotation values (top-left 3x3 of the 4x4 matrix)
    // Expected matrix structure (row-major):
    // [ r00 r01 r02 tx ]
    // [ r10 r11 r12 ty ]
    // [ r20 r21 r22 tz ]
    // [  0   0   0  1 ]
    const double r00 = flat[off + 0],  r01 = flat[off + 1],  r02 = flat[off + 2];
    const double r10 = flat[off + 4],  r11 = flat[off + 5],  r12 = flat[off + 6];
    const double r20 = flat[off + 8],  r21 = flat[off + 9],  r22 = flat[off +10];

    // Build the 3x3 rotation matrix
    Eigen::Matrix3d R;
    R << r00, r01, r02,
         r10, r11, r12,
         r20, r21, r22;

    // --- Orthonormalization with SVD ---
    // Due to numerical errors, R may not be perfectly orthogonal.
    // U*V^T is the projection of R onto the orthogonal group (rotations or reflections).
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU(), V = svd.matrixV();
    Eigen::Matrix3d R_ortho = U * V.transpose();

    // If determinant is negative, the matrix is a reflection (det = -1).
    // In that case flip the sign of one column of U to get a true rotation (det = +1).
    if (R_ortho.determinant() < 0) {
        U.col(2) *= -1.0;
        R_ortho = U * V.transpose();
    }

    // Convert the orthonormal matrix to a quaternion
    out_q = Eigen::Quaterniond(R_ortho);

    // Normalize quaternion for safety (norm = 1)
    out_q.normalize();

    return true; // success
}

// =============================================================================
// start() — startup of ROS2, TF2, YARP ports and ROS2 service
// =============================================================================
bool ExecuteDanceComponent::start(int argc, char* argv[])
{
    // 1) Initialize ROS2 if needed
    if (!rclcpp::ok()) {
        rclcpp::init(argc, argv);
    }

    // 2) Start the controllers if they are not already active (assumes YARP network is already initialized)
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

    // 3) Wait for the RPC servers to appear (no timeout: convenient for development)
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Waiting for port %s...", cartesianPortLeft.c_str());
    while (!yarp::os::Network::exists(cartesianPortLeft)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Waiting for port %s...", cartesianPortRight.c_str());
    while (!yarp::os::Network::exists(cartesianPortRight)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 4) Create the ROS2 node and persistent TF2 (buffer + listener)
    m_node = rclcpp::Node::make_shared("ExecuteDanceComponentNode");
    m_tfBuffer   = std::make_unique<tf2_ros::Buffer>(m_node->get_clock());
    m_tfListener = std::make_unique<tf2_ros::TransformListener>(*m_tfBuffer);

    // 5) Load artwork coordinates (in frame "map")
    m_artworkCoords = loadArtworkCoordinates("/home/user1/UC3/conf/artwork_coords.json");

    // 6) Open and connect local RPC client ports to the two cartesian controllers
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

    // 7) ROS2 service
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
// close() — close ports and shut down ROS2
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
// executeTask() — core logic of pointing (posizione-only)
// =============================================================================
void ExecuteDanceComponent::executeTask(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request)
{
    RCLCPP_DEBUG(m_node->get_logger(), "EXECUTE TASK: '%s'", request->dance_name.c_str());
    // deleteAllMarkers();

    // 1) Find artwork coordinates by name (in frame "map") from json file
    auto it = m_artworkCoords.find(request->dance_name);
    if (it == m_artworkCoords.end() || it->second.size() < 3) {
        RCLCPP_WARN(m_node->get_logger(), "No or invalid ARTWORK '%s' found", request->dance_name.c_str());
        return;
    }
    const auto& coords = it->second;

    // 2) Transform the point from map frame to  base_link frame
    //    (if TF fails, use the given coordinates as-is in base_link)

    Eigen::Vector3d p_base{coords[0], coords[1], coords[2]};
    geometry_msgs::msg::Point mapPt; mapPt.x = coords[0]; mapPt.y = coords[1]; mapPt.z = coords[2];
    geometry_msgs::msg::Point basePt;

    // RCLCPP_DEBUG(m_node->get_logger(),
    //             "[TARGET] '%s' in %s : (%.3f, %.3f, %.3f)",
    //             request->dance_name.c_str(), m_mapFrame.c_str(),
    //             mapPt.x, mapPt.y, mapPt.z);

    if (transformPointMapToRobot(mapPt, basePt, m_baseFrame, 1.0)) {
        p_base = Eigen::Vector3d(basePt.x, basePt.y, basePt.z);
        // RCLCPP_DEBUG(m_node->get_logger(),
        //             "[TARGET] '%s' in %s : (%.3f, %.3f, %.3f)",
        //             request->dance_name.c_str(), m_baseFrame.c_str(),
        //             p_base.x(), p_base.y(), p_base.z());
    } else {
        RCLCPP_WARN(m_node->get_logger(),
                    "TF map->%s failed; using given coords as %s",
                    m_baseFrame.c_str(), m_baseFrame.c_str());
        RCLCPP_DEBUG(m_node->get_logger(),
                    "[TARGET] fallback in %s : (%.3f, %.3f, %.3f)",
                    m_baseFrame.c_str(), p_base.x(), p_base.y(), p_base.z());
    }

    // 3) Pre-scan: query get_pose for both arms, estimate distances from target
    std::vector<std::pair<std::string,double>> armDistances;                 // (arm,  distance)
    std::map<std::string, std::vector<double>> cachedPoseValues;             // (arm -> flat pose)
    if (!preScanArticulatedArms(p_base, armDistances, cachedPoseValues)) {
        RCLCPP_ERROR(m_node->get_logger(), "No arms available for pointing");
        return;
    }

    // 4) Sort arms by distance from target (closest first), prefer arm by torso Y sign
    //    (if torso Y>0 prefer LEFT, else RIGHT; fallback: LEFT) --- IGNORE ---
    std::string preferredArm;
    if (!chooseArmByTorsoY(p_base, preferredArm)) preferredArm = "LEFT";     // fallback
    std::stable_sort(armDistances.begin(), armDistances.end(), [&](auto&a, auto&b){
        if (a.first==preferredArm && b.first!=preferredArm) return true;     
        if (a.first!=preferredArm && b.first==preferredArm) return false;
        return a.second < b.second;                                           
    });

    // 5) Try to reach the candidate point with each arm in order of preference until success 
    for (const auto &armEntry : armDistances) {
        const std::string armName = armEntry.first;                           // "LEFT" o "RIGHT"
        yarp::os::Port* activePort = (armName == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
        const std::string remotePort = (armName == "LEFT") ? "/r1-cartesian-control/left_arm/rpc:i"
                                                           : "/r1-cartesian-control/right_arm/rpc:i";
        if (!activePort->isOpen() || !yarp::os::Network::isConnected(activePort->getName(), remotePort)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: port not ready, skipping", armName.c_str());
            continue;
        }

        // 5.a)  Get shoulder/targets geometry
        Eigen::Vector3d shoulder_base;
        if (!getShoulderPosInBase(armName, shoulder_base)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: missing shoulder TF", armName.c_str());
            continue;
        }
        //  Candidate point on the shoulder→target line (within reach sphere)
        Eigen::Vector3d candidate = sphereReachPoint(shoulder_base, p_base);

        // 5.b)  Extract current orientation from cached pose (fallback: identity)
        Eigen::Quaterniond q_keep(1,0,0,0); // identità (fallback)
        {
            auto itPose = cachedPoseValues.find(armName);
            if (itPose != cachedPoseValues.end()) {
                Eigen::Quaterniond q_tmp;
                if (quatFromFlatPose(itPose->second, q_tmp)) {
                    q_keep = q_tmp;
                } else {
                    RCLCPP_WARN(m_node->get_logger(), "ARM %s: cannot parse current orientation, using identity", armName.c_str());
                }
            } else {
                RCLCPP_WARN(m_node->get_logger(), "ARM %s: no cached pose, using identity orientation", armName.c_str());
            }
        }

        // 5.c) Reachability with current orientation (keep ori) 
        if (!isPoseReachable(activePort, candidate, q_keep)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: candidate not reachable (kept ori), trying next arm", armName.c_str());
            continue;
        }

        // 5.d)  Move to candidate point keeping current orientation
        RCLCPP_DEBUG(m_node->get_logger(), "ARM %s: moving to (%.3f, %.3f, %.3f) keeping orientation",
                     armName.c_str(), candidate.x(), candidate.y(), candidate.z());
        yarp::os::Bottle cmd_pose_final, res_pose_final;
        cmd_pose_final.addString("go_to_pose");
        cmd_pose_final.addFloat64(candidate.x());
        cmd_pose_final.addFloat64(candidate.y());
        cmd_pose_final.addFloat64(candidate.z());
        cmd_pose_final.addFloat64(q_keep.x());
        cmd_pose_final.addFloat64(q_keep.y());
        cmd_pose_final.addFloat64(q_keep.z());
        cmd_pose_final.addFloat64(q_keep.w());
        cmd_pose_final.addFloat64(5.0); // traj time (s) 

        bool ok = activePort->write(cmd_pose_final, res_pose_final);
        if (ok && res_pose_final.size()>0 && res_pose_final.get(0).asVocab32()==yarp::os::createVocab32('o','k')) {
            RCLCPP_DEBUG(m_node->get_logger(), "SUCCESS: %s moved to candidate keeping orientation", armName.c_str());
            break;
        } else {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: go_to_pose failed, trying next arm", armName.c_str());
            continue;
        }
    }
}

// =============================================================================
// loadArtworkCoordinates() — loads { "artworks": { name: {x,y,z} } }
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
            RCLCPP_DEBUG(m_node->get_logger(), "Loaded artwork '%s' [%.2f %.2f %.2f]", name.c_str(), x,y,z);
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(m_node->get_logger(), "Error parsing '%s': %s", filename.c_str(), ex.what());
    }
    return artworkMap;
}



// =============================================================================
// transformPointMapToRobot() — uses persistent buffer/listener
// =============================================================================
bool ExecuteDanceComponent::transformPointMapToRobot(const geometry_msgs::msg::Point& map_point,
                                                     geometry_msgs::msg::Point& out_robot_point,
                                                     const std::string& robot_frame,
                                                     double timeout_sec)
{
    if (!m_tfBuffer) {
        RCLCPP_WARN(m_node->get_logger(), "TF buffer not initialized");
        return false;
    }

    const std::string map_frame = m_mapFrame; // typically "map"
    const auto timeout = tf2::durationFromSec(timeout_sec);

    if (!m_tfBuffer->canTransform(robot_frame, map_frame, tf2::TimePointZero, timeout)) {
        RCLCPP_WARN(m_node->get_logger(),
                    "TF: transform %s <- %s not available (request: map:[%.3f %.3f %.3f])",
                    robot_frame.c_str(), map_frame.c_str(),
                    map_point.x, map_point.y, map_point.z);
        return false;
    }

    try {
        auto tf_stamped = m_tfBuffer->lookupTransform(robot_frame, map_frame,
                                                      tf2::TimePointZero, timeout);

        geometry_msgs::msg::PointStamped p_in, p_out;
        p_in.header = tf_stamped.header;
        p_in.header.frame_id = map_frame;
        p_in.point = map_point;

        tf2::doTransform(p_in, p_out, tf_stamped);
        out_robot_point = p_out.point;

        RCLCPP_DEBUG(m_node->get_logger(),
                    "[TF] %s <- %s : (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f)",
                    robot_frame.c_str(), map_frame.c_str(),
                    map_point.x, map_point.y, map_point.z,
                    out_robot_point.x, out_robot_point.y, out_robot_point.z);
        return true;

    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(m_node->get_logger(), "TF exception: %s", ex.what());
        return false;
    }
}

// =============================================================================
// preScanArticulatedArms() — queries get_pose and estimates distances from target
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
// getTFMatrix(), transformPoint(), chooseArmByTorsoY(), getShoulderPosInBase(),
// =============================================================================

bool ExecuteDanceComponent::getTFMatrix(const std::string& target,
                                        const std::string& source,
                                        Eigen::Matrix4d& T) const
{
    // If the TF buffer is not initialized, fail
    if (!m_tfBuffer) return false;
    try {
        // Query the transform between frames (target <- source)
        auto tf = m_tfBuffer->lookupTransform(target, source, rclcpp::Time(0), std::chrono::seconds(1));

        // Extract translation and rotation (as quaternion) from the transform
        const auto& tr = tf.transform.translation;
        const auto& q  = tf.transform.rotation;

        // Build Eigen quaternion and convert to rotation matrix
        Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);
        Eigen::Matrix3d R = Q.toRotationMatrix();

        // Fill the 4x4 homogeneous transformation matrix
        T.setIdentity();
        T.topLeftCorner<3,3>() = R;
        T(0,3)=tr.x; T(1,3)=tr.y; T(2,3)=tr.z;

        return true; // success
    } catch (const tf2::TransformException& e) {
        // If the transform lookup fails, log warning and return false
        RCLCPP_WARN(m_node->get_logger(), "TF lookup failed %s<- %s : %s",
                    target.c_str(), source.c_str(), e.what());
        return false;
    }
}

Eigen::Vector3d ExecuteDanceComponent::transformPoint(const Eigen::Matrix4d& T,
                                                      const Eigen::Vector3d& p)
{
    // Convert point to homogeneous coordinates [x,y,z,1]
    Eigen::Vector4d ph; ph << p, 1.0;

    // Apply transformation matrix
    ph = T*ph;

    // Return only the first 3 components (x,y,z)
    return ph.head<3>();
}

bool ExecuteDanceComponent::chooseArmByTorsoY(const Eigen::Vector3d& p_base,
                                              std::string& outArm) const
{
    // Transformation from base -> torso
    Eigen::Matrix4d T_torso_base;
    if (!getTFMatrix(m_torsoFrame, m_baseFrame, T_torso_base)) return false;

    // Express the target point in the torso frame
    Eigen::Vector3d p_torso = transformPoint(T_torso_base, p_base);

    // Choose LEFT arm if target is on positive Y side in torso frame, else RIGHT
    outArm = (p_torso.y() > 0.0) ? "LEFT" : "RIGHT";
    return true;
}

bool ExecuteDanceComponent::getShoulderPosInBase(const std::string& arm,
                                                 Eigen::Vector3d& out_pos) const
{
    // Pick the correct shoulder frame based on arm side
    const std::string& shoulder = (arm=="LEFT") ? m_lShoulderFrame : m_rShoulderFrame;

    // Get transform from base -> shoulder
    Eigen::Matrix4d T_base_sh;
    if (!getTFMatrix(m_baseFrame, shoulder, T_base_sh)) return false;

    // Extract translation vector (position of shoulder in base frame)
    out_pos = T_base_sh.block<3,1>(0,3);
    return true;
}

// =============================================================================
// sphereReachPoint() — candidate on the shoulder→target line
// =============================================================================
Eigen::Vector3d ExecuteDanceComponent::sphereReachPoint(const Eigen::Vector3d& shoulder_base,
                                                        const Eigen::Vector3d& target_base) const
{
    // Distance from shoulder to target
    const double d = (target_base - shoulder_base).norm();

    // Maximum reach radius: clamp to [minDist, reachRadius]
    const double R = std::min(m_reachRadius, std::max(d, m_minDist));

    // Effective reach length: slightly back off from the target by safetyBackoff
    const double L = std::min(std::max(d - m_safetyBackoff, 0.0), R);

    // Degenerate case: if distance is ~0, return shoulder position
    if (d < 1e-6) return shoulder_base;

    // Direction vector shoulder -> target (normalized)
    const Eigen::Vector3d dir = (target_base - shoulder_base) / d;

    // Candidate point = shoulder + L * dir
    return shoulder_base + L * dir;
}
