/****************************************************************************
 * UC3 — CartesianPointingComponent (implementation)
 *
 * Minimal VARIANT: keeps behavior unchanged but loads artwork list using
 * YARP ResourceFinder (context + --artworks). No other functional changes.
 ****************************************************************************/
#include "CartesianPointingComponent.h"

// C++ standard
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cmath>
#include <functional>

// JSON (nlohmann)
#include <nlohmann/json.hpp>
using ordered_json = nlohmann::ordered_json;

// TF2 LinearMath (optional utilities)
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// =============================================================================
// Helper: build a quaternion from the flat 4x4 pose array returned by controller
// The expected layout (row-major) is:
// [ r00 r01 r02 tx  r10 r11 r12 ty  r20 r21 r22 tz  0 0 0 1 ]
// This utility extracts the 3x3 rotation, orthonormalizes it (SVD) and returns
// a normalized Eigen::Quaterniond.
static bool quatFromFlatPose(const std::vector<double>& flat, Eigen::Quaterniond& out_q)
{
    // Only accept flattened poses of size 16 (4x4 matrix) or 18 (extra 2 values at the beginning).
    if (flat.size() != 16 && flat.size() != 18)
        return false;

    // If there are 18 values, skip the first two (often metadata like timestamp, id, etc.).
    const size_t off = (flat.size() == 18) ? 2u : 0u;

    // Extract the 3x3 rotation submatrix from the flattened 4x4 transformation matrix.
    const double r00 = flat[off + 0],  r01 = flat[off + 1],  r02 = flat[off + 2];
    const double r10 = flat[off + 4],  r11 = flat[off + 5],  r12 = flat[off + 6];
    const double r20 = flat[off + 8],  r21 = flat[off + 9],  r22 = flat[off +10];

    Eigen::Matrix3d R;
    R << r00, r01, r02,
         r10, r11, r12,
         r20, r21, r22;

    // Numerical errors may cause R to not be perfectly orthonormal (e.g., after long computations).
    // We orthonormalize it using Singular Value Decomposition (SVD),
    // projecting R onto the closest valid rotation matrix.
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU(), V = svd.matrixV();
    Eigen::Matrix3d R_ortho = U * V.transpose();

    // Ensure the result is a proper rotation (determinant = +1, not -1).
    // If det < 0, it means we have a reflection; flip the last column to correct it.
    if (R_ortho.determinant() < 0) { 
        U.col(2) *= -1.0; 
        R_ortho = U * V.transpose(); 
    }

    // Convert the orthonormal rotation matrix into a quaternion.
    // Quaternions are compact and avoid issues like gimbal lock.
    out_q = Eigen::Quaterniond(R_ortho);

    // Normalize the quaternion to guarantee it has unit length (safe against drift).
    out_q.normalize();

    return true;
}


// =============================================================================
// start() — initialize ROS2, TF2, YARP ports, services and load artworks via RF
// =============================================================================
bool CartesianPointingComponent::start(int argc, char* argv[])
{
    // 1) Initialize ROS2 if needed.
    //    We guard rclcpp::init so start() can be safely called from both
    //    standalone binaries and larger systems that may have already initialized ROS2.
    if (!rclcpp::ok()) {
        rclcpp::init(argc, argv);
    }

    // Do not start controllers from the component process.
    // Controllers must be launched externally by the operator or system service.
    // If the expected RPC ports are missing we log a clear instruction so the
    // operator can start them and then the component will wait for availability.
    if (!yarp::os::Network::exists(cartesianPortLeft)) {
        RCLCPP_WARN(m_node ? m_node->get_logger() : rclcpp::get_logger("rclcpp"),
                    "Left Cartesian controller not present. Please start it externally:\n  r1-cartesian-control --from ");
    }
    if (!yarp::os::Network::exists(cartesianPortRight)) {
        RCLCPP_WARN(m_node ? m_node->get_logger() : rclcpp::get_logger("rclcpp"),
                    "Right Cartesian controller not present. Please start it externally:\n  r1-cartesian-control --from ");
    }
    //    (A sleep prevents tight-loop polling and reduces CPU usage.)
    // Wait indefinitely for the RPC server ports to appear.
    // The controller is a separate process and must be launched by the operator.
    RCLCPP_INFO(m_node ? m_node->get_logger() : rclcpp::get_logger("rclcpp"),
                 "Waiting for LEFT controller port '%s'... (will wait indefinitely)", cartesianPortLeft.c_str());
    while (!yarp::os::Network::exists(cartesianPortLeft)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(m_node ? m_node->get_logger() : rclcpp::get_logger("rclcpp"),
                 "Waiting for RIGHT controller port '%s'... (will wait indefinitely)", cartesianPortRight.c_str());
    while (!yarp::os::Network::exists(cartesianPortRight)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 4) Create ROS2 node and a persistent TF2 buffer+listener.
    //    A long-lived buffer allows time-synchronized lookups and reduces repeated allocations.
    m_node = rclcpp::Node::make_shared("CartesianPointingComponentNode");
    m_tfBuffer   = std::make_unique<tf2_ros::Buffer>(m_node->get_clock());
    m_tfListener = std::make_unique<tf2_ros::TransformListener>(*m_tfBuffer);

    // 5) Load artworks via YARP ResourceFinder (context + --artworks parameter).
    //    Using RF decouples file locations from code and supports deployment contexts.
    {
        yarp::os::ResourceFinder rf;
        rf.setDefaultContext(m_rfDefaultContext);  // default search context (override with --context)
        rf.setDefault("artworks", m_rfDefaultArtworks); // default filename (override with --artworks)
        rf.configure(argc, argv); // parse CLI to allow overrides

        // Resolve the filename to an absolute path according to YARP_DATA_DIRS and context.
        const std::string artworksName = rf.check("artworks",
            yarp::os::Value(m_rfDefaultArtworks)).asString();

        const std::string artworksPath = rf.findFileByName(artworksName);
        if (artworksPath.empty()) {
            // Fail fast with a helpful diagnostic including the search path.
            const char* yarp_dirs = std::getenv("YARP_DATA_DIRS");
            RCLCPP_ERROR(m_node ? m_node->get_logger() : rclcpp::get_logger("rclcpp"),
                        "Artwork file '%s' not found in context '%s'. Searched YARP_DATA_DIRS='%s'. Aborting startup.",
                        artworksName.c_str(), rf.getContext().c_str(),
                        yarp_dirs ? yarp_dirs : "(unset)");
            return false; // Without an artwork list, the component cannot fulfill its purpose.
        } else {
            // Preload coordinates so subsequent calls do not incur IO/parsing cost.
            m_artworkCoords = loadArtworkCoordinates(artworksPath);
        }
    }

    // 6) Open and connect local YARP client ports to the cartesian controllers.
    //    We disconnect any stale connections first to ensure a clean routing state.
    if (yarp::os::Network::exists(m_cartesianPortNameLeft)) {
        yarp::os::Network::disconnect(m_cartesianPortNameLeft, cartesianPortLeft);
        std::this_thread::sleep_for(std::chrono::milliseconds(300)); // allow routing tables to settle
    }
    if (!m_cartesianPortLeft.open(m_cartesianPortNameLeft)) {
        // Fail early: without the client port, we cannot issue RPC to the controller.
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

    // 7) Create ROS2 services.
    //    Exposing an RPC-like service interface lets external nodes trigger point-at tasks
    //    without coupling to YARP internals, bridging ecosystems cleanly.
    m_srvPointAt = m_node->create_service<cartesian_pointing_interfaces::srv::PointAt>(
        "/CartesianPointingComponent/PointAt",
        [this](const std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Request> request,
               std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Response> response)
        {
            // Delegate to the main task handler; any internal error handling remains localized.
            this->pointTask(request);
            // For now we always report success; refine if pointTask sets detailed statuses.
            response->is_ok = true;
            response->error_msg = "";
        }
    );

    // Final readiness message to help operators/devs confirm successful startup in logs.
    RCLCPP_DEBUG(m_node->get_logger(), "CartesianPointingComponent READY");
    return true;
}


// =============================================================================
// close() — close ports and shutdown ROS2
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
// pointTask() — main pointing routine (position-only variant)
// Steps:
// 1) find artwork coordinates by name
// 2) transform artwork from map -> base (if TF available)
// 3) pre-scan arm poses and estimate distances
// 4) choose and sort arms by preference
// 5) try candidate points and command controller
// =============================================================================
void CartesianPointingComponent::pointTask(const std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Request> request)
{
    RCLCPP_DEBUG(m_node->get_logger(), "EXECUTE TASK: '%s'", request->target_name.c_str());

    // 1) Lookup artwork coordinates by name
    auto it = m_artworkCoords.find(request->target_name);
    if (it == m_artworkCoords.end() || it->second.size() < 3) {
        RCLCPP_WARN(m_node->get_logger(), "No or invalid TARGET '%s' found", request->target_name.c_str());
        return;
    }
    const auto& coords = it->second;

    // 2) Transform the point from map frame to base frame 
    Eigen::Vector3d p_base{coords[0], coords[1], coords[2]};
    geometry_msgs::msg::Point mapPt; mapPt.x = coords[0]; mapPt.y = coords[1]; mapPt.z = coords[2];
    geometry_msgs::msg::Point basePt;
    if (transformPointMapToRobot(mapPt, basePt, m_baseFrame, 1.0)) {
        p_base = Eigen::Vector3d(basePt.x, basePt.y, basePt.z);
    } else {
        RCLCPP_WARN(m_node->get_logger(), "TF map->%s failed; using given coords as %s",
                    m_baseFrame.c_str(), m_baseFrame.c_str());
    }

    // 3) Pre-scan both arms to get poses and distances
    std::vector<std::pair<std::string,double>> armDistances;
    std::map<std::string, std::vector<double>> cachedPoseValues;
    if (!preScanArticulatedArms(p_base, armDistances, cachedPoseValues)) {
        RCLCPP_ERROR(m_node->get_logger(), "No arms available for pointing");
        return;
    }

    // 4) Determine preferred arm by torso Y and sort by distance
    std::string preferredArm;
    if (!chooseArmByTorsoY(p_base, preferredArm)) preferredArm = "LEFT";
    std::stable_sort(armDistances.begin(), armDistances.end(), [&](auto&a, auto&b){
        if (a.first==preferredArm && b.first!=preferredArm) return true;
        if (a.first!=preferredArm && b.first==preferredArm) return false;
        return a.second < b.second;
    });

    // mark pointing flag
    {
        std::lock_guard<std::mutex> lk(m_flagMutex);
        m_isPointing = true;
    }

    // 5) Try each arm in order until one succeeds
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

        // compute candidate point on the shoulder->target line within reach sphere
        Eigen::Vector3d candidate = sphereReachPoint(shoulder_base, p_base);

        // extract current end-effector orientation from cached pose; fallback to identity
        Eigen::Quaterniond q_keep(1,0,0,0);
        if (auto itPose = cachedPoseValues.find(armName); itPose != cachedPoseValues.end()) {
            Eigen::Quaterniond q_tmp; if (quatFromFlatPose(itPose->second, q_tmp)) q_keep = q_tmp;
        }

        // check reachability using controller RPC
        if (!isPoseReachable(activePort, candidate, q_keep)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: candidate not reachable, trying next arm", armName.c_str());
            continue;
        }

        // Prepare and log the go_to_pose command we'll send
        yarp::os::Bottle cmd, res;
        cmd.addString("go_to_pose");
        cmd.addFloat64(candidate.x());
        cmd.addFloat64(candidate.y());
        cmd.addFloat64(candidate.z());
        cmd.addFloat64(q_keep.x());
        cmd.addFloat64(q_keep.y());
        cmd.addFloat64(q_keep.z());
        cmd.addFloat64(q_keep.w());
        cmd.addFloat64(5.0); // trajectory duration (s)

        // Log which arm, which command, and which artwork target we are pointing at
        RCLCPP_INFO(m_node->get_logger(), "POINTING: arm=%s cmd=%s target='%s' candidate=(%.3f, %.3f, %.3f)",
                    armName.c_str(), cmd.get(0).asString().c_str(), request->target_name.c_str(),
                    candidate.x(), candidate.y(), candidate.z());

        bool ok = activePort->write(cmd, res);
        if (ok && res.size()>0 && res.get(0).asVocab32()==yarp::os::createVocab32('o','k')) {
            RCLCPP_DEBUG(m_node->get_logger(), "SUCCESS: %s moved to candidate keeping orientation", armName.c_str());
            break; // success with one arm
        } else {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: go_to_pose failed, trying next arm", armName.c_str());
            continue;
        }
    }

    // clear pointing flag
    { std::lock_guard<std::mutex> lk(m_flagMutex); m_isPointing = false; }
}

// =============================================================================
// loadArtworkCoordinates() — load artwork entries { name : [x,y,z] }
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
            // Log loaded artwork name and coordinates
            RCLCPP_DEBUG(m_node->get_logger(),
                        "Loaded artwork '%s' -> (%.3f, %.3f, %.3f)",
                        name.c_str(), x, y, z);
        }

        // If none found, warn the caller
        if (artworkMap.empty()) {
            RCLCPP_WARN(m_node->get_logger(), "No artwork entries found in '%s'", filename.c_str());
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(m_node->get_logger(), "Error parsing '%s': %s", filename.c_str(), ex.what());
    }
    return artworkMap;
}

// =============================================================================
// transformPointMapToRobot() — transform a point using persistent TF2 buffer
// =============================================================================
bool CartesianPointingComponent::transformPointMapToRobot(const geometry_msgs::msg::Point& map_point,
                                                          geometry_msgs::msg::Point& out_robot_point,
                                                          const std::string& robot_frame,
                                                          double timeout_sec)
{
    // We need a TF buffer to query the TF tree; if it's missing, we cannot transform anything.
    if (!m_tfBuffer) {
        RCLCPP_WARN(m_node->get_logger(), "TF buffer not initialized");
        return false;
    }

    const std::string map_frame = m_mapFrame;  // Keep a stable copy (could be a parameter).
    const auto timeout = tf2::durationFromSec(timeout_sec);  // Respect caller's timing constraints.

    // Before attempting the transform, check its availability.
    // This avoids throwing exceptions and lets us log a clean diagnostic.
    if (!m_tfBuffer->canTransform(robot_frame,        // target frame (where we want the point)
                                  map_frame,          // source frame (where the point currently is)
                                  tf2::TimePointZero, // "latest available" common time
                                  timeout))
    {
        RCLCPP_WARN(m_node->get_logger(), "TF: transform %s <- %s not available",
                    robot_frame.c_str(), map_frame.c_str());
        return false;
    }

    try {
        // Look up the transform once we're confident it's available.
        // Using TimePointZero requests the most recent transform, which is typical for live robots.
        auto tf_stamped = m_tfBuffer->lookupTransform(robot_frame, map_frame,
                                                      tf2::TimePointZero, timeout);

        // tf2 operates on stamped types to carry frame_id and timestamp;
        // wrap the raw Point into a PointStamped providing the proper header.
        geometry_msgs::msg::PointStamped p_in, p_out;
        p_in.header = tf_stamped.header;  // Keep a consistent timestamp reference.
        p_in.header.frame_id = map_frame; // Explicitly set the source frame of the input point.
        p_in.point = map_point;

        // Apply the transform computed above.
        // Using doTransform ensures consistent handling of translation/rotation and timestamp/frame metadata.
        tf2::doTransform(p_in, p_out, tf_stamped);

        // Return just the geometry (the caller asked for a Point, not a PointStamped).
        out_robot_point = p_out.point;
        return true;

    } catch (const tf2::TransformException &ex) {
        // Transform lookup or application can still fail (e.g., timeout, extrapolation).
        // Catch and report the specific TF error for easier debugging.
        RCLCPP_WARN(m_node->get_logger(), "TF exception: %s", ex.what());
        return false;
    }
}

// =============================================================================
// preScanArticulatedArms() — query get_pose from controllers and compute distances
// =============================================================================
bool CartesianPointingComponent::preScanArticulatedArms(const Eigen::Vector3d& artwork_pos,
                                                        std::vector<std::pair<std::string,double>>& armDistances,
                                                        std::map<std::string, std::vector<double>>& cachedPoseValues)
{
    // Clear outputs at the start so callers never see stale data if we early-return.
    armDistances.clear();
    cachedPoseValues.clear();

    // Iterate over both arms in a fixed order to keep outputs deterministic.
    for (const std::string armId : {"LEFT", "RIGHT"}) {
        // Select the correct client/server ports per arm.
        // Using explicit names avoids accidental cross-wiring between limbs.
        yarp::os::Port* clientPort = (armId == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
        const std::string serverPort = (armId == "LEFT") ? "/r1-cartesian-control/left_arm/rpc:i"
                                                         : "/r1-cartesian-control/right_arm/rpc:i";

        // Fast pre-check: only attempt RPC if the client is open and the route exists.
        // This avoids blocking calls and produces clearer diagnostics.
        if (!clientPort->isOpen() || !yarp::os::Network::isConnected(clientPort->getName(), serverPort)) {
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: %s port not ready", armId.c_str());
            continue;
        }

        // Ask the Cartesian controller for the current end-effector pose.
        // We use "get_pose" because it is a lightweight, read-only query.
        yarp::os::Bottle cmd_get, res_get;
        cmd_get.addString("get_pose");
        if (!clientPort->write(cmd_get, res_get)) {
            // If the RPC fails, skip this arm instead of failing the whole call:
            // partial results are still useful to the caller.
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: get_pose failed for %s", armId.c_str());
            continue;
        }

        // Flatten the returned Bottle into a simple vector<double>.
        // Different servers may nest values (e.g., [R t] as lists) or return [status, data].
        // We try a tolerant strategy: (1) flatten the whole bottle; (2) if size is not 16/18,
        // search recursively for the first nested list that flattens to 16 or 18 numbers.
        std::vector<double> flat_pose;
        flat_pose.reserve(18);

        // Recursive flattener: collect numeric scalars from any nested Bottle
        std::function<void(const yarp::os::Bottle&)> flatten = [&](const yarp::os::Bottle& b) {
            for (size_t i = 0; i < b.size(); ++i) {
                if (b.get(i).isList()) {
                    yarp::os::Bottle* sub = b.get(i).asList();
                    flatten(*sub);
                } else {
                    // yarp::os::Value does not provide uniform isInt()/asInt() across versions.
                    // Use safe extraction: prefer asFloat64(), but handle integer-like values as well.
                    double v = 0.0;
                    if (b.get(i).isFloat64()) {
                        v = b.get(i).asFloat64();
                    } else if (b.get(i).isInt8()) {
                        v = static_cast<double>(b.get(i).asInt8());
                    } else if (b.get(i).isInt16()) {
                        v = static_cast<double>(b.get(i).asInt16());
                    } else if (b.get(i).isInt32()) {
                        v = static_cast<double>(b.get(i).asInt32());
                    } else if (b.get(i).isInt64()) {
                        v = static_cast<double>(b.get(i).asInt64());
                    } else {
                        // Fallback: try asString then parse double; if that fails, skip value.
                        std::string s = b.get(i).toString();
                        try { v = std::stod(s); } catch (...) { continue; }
                    }
                    flat_pose.push_back(v);
                }
            }
        };

        flatten(res_get);

        // If the flattened top-level result does not contain the expected matrix,
        // look for the first nested list that does.
        if (flat_pose.size() != 16 && flat_pose.size() != 18) {
            bool found = false;
            std::function<bool(const yarp::os::Bottle&)> find_good_list = [&](const yarp::os::Bottle& b) -> bool {
                for (size_t i = 0; i < b.size(); ++i) {
                    if (b.get(i).isList()) {
                        yarp::os::Bottle* sub = b.get(i).asList();
                        std::vector<double> tmp;
                        std::function<void(const yarp::os::Bottle&)> f2 = [&](const yarp::os::Bottle& sb) {
                            for (size_t j = 0; j < sb.size(); ++j) {
                                if (sb.get(j).isList()) f2(*sb.get(j).asList());
                                else {
                                    double v = 0.0;
                                    if (sb.get(j).isFloat64()) {
                                        v = sb.get(j).asFloat64();
                                    } else if (sb.get(j).isInt8()) {
                                        v = static_cast<double>(sb.get(j).asInt8());
                                    } else if (sb.get(j).isInt16()) {
                                        v = static_cast<double>(sb.get(j).asInt16());
                                    } else if (sb.get(j).isInt32()) {
                                        v = static_cast<double>(sb.get(j).asInt32());
                                    } else if (sb.get(j).isInt64()) {
                                        v = static_cast<double>(sb.get(j).asInt64());
                                    } else {
                                        std::string s = sb.get(j).toString();
                                        try { v = std::stod(s); } catch (...) { continue; }
                                    }
                                    tmp.push_back(v);
                                }
                            }
                        };
                        f2(*sub);
                        if (tmp.size() == 16 || tmp.size() == 18) { flat_pose = std::move(tmp); return true; }
                        // recurse deeper
                        if (find_good_list(*sub)) return true;
                    }
                }
                return false;
            };
            found = find_good_list(res_get);
            if (!found) {
                // Try a few times: controllers may not have populated the full pose immediately.
                const int max_retries = 5;
                const std::chrono::milliseconds retry_delay(200);
                bool retried_ok = false;
                for (int attempt = 1; attempt <= max_retries && !retried_ok; ++attempt) {
                    std::this_thread::sleep_for(retry_delay);
                    yarp::os::Bottle res_retry;
                    if (!clientPort->write(cmd_get, res_retry)) continue;
                    // attempt to flatten and find a valid matrix in the newly returned bottle
                    flat_pose.clear();
                    flatten(res_retry);
                    if (flat_pose.size() == 16 || flat_pose.size() == 18) {
                        // success: use this result
                        res_get = res_retry;
                        retried_ok = true;
                        break;
                    }
                    // try nested search on the fresh result
                    bool found_retry = find_good_list(res_retry);
                    if (found_retry) {
                        // on success, find_good_list already populated flat_pose
                        res_get = res_retry;
                        retried_ok = true;
                        break;
                    }
                }

                if (!retried_ok) {
                    RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: unexpected pose size=%zu for %s (raw bottle size=%zu) after %d retries - falling back to shoulder heuristic",
                                flat_pose.size(), armId.c_str(), res_get.size(), max_retries);
                    // Fallback: use shoulder position (TF) as distance heuristic and store an identity pose
                    Eigen::Vector3d shoulder_base;
                    if (getShoulderPosInBase(armId, shoulder_base)) {
                        const double dist = (artwork_pos - shoulder_base).norm();
                        // store a canonical 4x4 identity matrix (row-major) so quatFromFlatPose can fallback to identity
                        std::vector<double> idmat(16, 0.0);
                        idmat[0] = 1.0; idmat[5] = 1.0; idmat[10] = 1.0; idmat[15] = 1.0;
                        armDistances.emplace_back(armId, dist);
                        cachedPoseValues.emplace(armId, std::move(idmat));
                        // continue to next arm (we already saved results)
                        continue;
                    } else {
                        RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: cannot obtain shoulder TF for %s, skipping arm", armId.c_str());
                        continue;
                    }
                }
            }
        }

        // Account for optional 2-value header so indices match the 4x4 layout.
        const size_t pose_offset = (flat_pose.size()==18) ? 2u : 0u;

        // Extract the translation (tx, ty, tz) from the flattened 4x4 matrix.
        // In row-major [ r00 r01 r02 tx ; r10 r11 r12 ty ; r20 r21 r22 tz ; 0 0 0 1 ],
        // translation sits at indices 3, 7, 11 relative to the start of the matrix block.
        const double hand_tx = flat_pose[pose_offset + 3];
        const double hand_ty = flat_pose[pose_offset + 7];
        const double hand_tz = flat_pose[pose_offset + 11];

        // Compute Euclidean distance from the end-effector to the target artwork position.
        // This gives a scalar ranking criterion to choose the most suitable arm later on.
        const double dist = (artwork_pos - Eigen::Vector3d(hand_tx, hand_ty, hand_tz)).norm();

        // Save both: (1) the distance (for selection/sorting) and
        // (2) the raw flattened pose (for subsequent reuse without another RPC).
        armDistances.emplace_back(armId, dist);
        cachedPoseValues.emplace(armId, std::move(flat_pose));
    }

    // Return whether we obtained at least one valid arm measurement.
    // Callers can decide how to proceed if only a subset was available.
    return !armDistances.empty();
}


// =============================================================================
// isPoseReachable() — RPC wrapper returning whether controller deems pose reachable
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

bool CartesianPointingComponent::getTFMatrix(const std::string& target,
                                             const std::string& source,
                                             Eigen::Matrix4d& T) const
{
    // A TF buffer is mandatory to perform lookups. If it's missing, we cannot proceed.
    if (!m_tfBuffer) return false;

    try {
        // Query the most recent transform available between 'source' and 'target'.
        // Using time=0 requests the latest common time; 1-second timeout gives robustness.
        auto tf = m_tfBuffer->lookupTransform(target, source,
                                              rclcpp::Time(0), std::chrono::seconds(1));

        // Extract translation and rotation (quaternion) from the transform.
        const auto& tr = tf.transform.translation;
        const auto& q  = tf.transform.rotation;

        // Convert quaternion to Eigen::Quaterniond (Eigen uses double precision).
        Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);

        // Get the 3x3 rotation matrix from the quaternion.
        Eigen::Matrix3d R = Q.toRotationMatrix();

        // Initialize T as identity before filling values.
        // This ensures we always return a well-formed homogeneous transform.
        T.setIdentity();

        // Top-left 3x3 block encodes rotation.
        T.topLeftCorner<3,3>() = R;

        // Last column encodes translation (x, y, z).
        T(0,3) = tr.x;
        T(1,3) = tr.y;
        T(2,3) = tr.z;

        return true;

    } catch (const tf2::TransformException& e) {
        // Transform lookup may fail due to unavailable frames, time issues, or timeout.
        // Log a warning so the caller can see which frames failed and why.
        RCLCPP_WARN(m_node->get_logger(), "TF lookup failed %s<- %s : %s",
                    target.c_str(), source.c_str(), e.what());
        return false;
    }
}


Eigen::Vector3d CartesianPointingComponent::transformPoint(const Eigen::Matrix4d& T,
                                                           const Eigen::Vector3d& p)
{
    // Use homogeneous coordinates to apply rotation + translation in one matrix multiply.
    Eigen::Vector4d ph; ph << p, 1.0;  // w = 1 ensures translation is applied.
    ph = T * ph;                       // Apply the rigid transform.
    return ph.head<3>();               // Drop homogeneous coordinate and return xyz only.
}


bool CartesianPointingComponent::chooseArmByTorsoY(const Eigen::Vector3d& p_base,
                                                   std::string& outArm) const
{
    Eigen::Matrix4d T_torso_base;
    // We need the point in the torso frame to decide "left vs right" consistently.
    if (!getTFMatrix(m_torsoFrame, m_baseFrame, T_torso_base)) return false;

    // Transform the point from base frame into torso frame.
    Eigen::Vector3d p_torso = transformPoint(T_torso_base, p_base);

    // Convention: +Y in torso frame is robot's left side, -Y is right side.
    // This makes the arm choice invariant to robot yaw relative to the world.
    outArm = (p_torso.y() > 0.0) ? "LEFT" : "RIGHT";
    return true;
}


bool CartesianPointingComponent::getShoulderPosInBase(const std::string& arm,
                                                      Eigen::Vector3d& out_pos) const
{
    // Select the correct shoulder frame for the queried arm.
    const std::string& shoulder = (arm == "LEFT") ? m_lShoulderFrame : m_rShoulderFrame;

    Eigen::Matrix4d T_base_sh;
    // We request the transform that maps points from shoulder frame into base frame.
    // Doing so lets us read the shoulder position directly in base coordinates.
    if (!getTFMatrix(m_baseFrame, shoulder, T_base_sh)) return false;

    // In a homogeneous transform, the last column (0..2,3) stores translation.
    // Extract it instead of multiplying a zero vector—cheaper and clearer.
    out_pos = T_base_sh.block<3,1>(0,3);
    return true;
}


// =============================================================================
// sphereReachPoint() — compute a point on the shoulder->target line within reach
// =============================================================================
Eigen::Vector3d CartesianPointingComponent::sphereReachPoint(const Eigen::Vector3d& shoulder_base,
                                                             const Eigen::Vector3d& target_base) const
{
    // Euclidean distance from shoulder to target.
    const double d = (target_base - shoulder_base).norm();

    // Effective reach radius:
    // - cannot exceed robot's max reach (m_reachRadius),
    // - cannot be smaller than a minimum distance (m_minDist),
    // - clamps between those two bounds.
    const double R = std::min(m_reachRadius, std::max(d, m_minDist));

    // Length along the shoulder->target direction:
    // - start slightly before target (safety backoff),
    // - never negative,
    // - and capped at the reach radius R.
    const double L = std::min(std::max(d - m_safetyBackoff, 0.0), R);

    // If target coincides with shoulder (degenerate case),
    // just return the shoulder position to avoid division by zero.
    if (d < 1e-6) return shoulder_base;

    // Normalize the shoulder->target vector to get direction.
    const Eigen::Vector3d dir = (target_base - shoulder_base) / d;

    // Return a point along that line, at distance L from shoulder.
    return shoulder_base + L * dir;
}


