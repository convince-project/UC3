// =========================
// CartesianPointingComponent.cpp
// =========================
#include "CartesianPointingComponent.h"

#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cmath>
#include <nlohmann/json.hpp>

// =============================================================
// Small YARP helpers to interpret RPC replies from the controller
// =============================================================

// Convert a 1-element Bottle to bool (accepts bool/int/double conventions)
static bool bottleAsBool(const yarp::os::Bottle& b) {
    if (b.size()==0) return false;
    if (b.get(0).isBool())    return b.get(0).asBool();
    if (b.get(0).isInt32())   return b.get(0).asInt32()!=0;
    if (b.get(0).isFloat64()) return std::abs(b.get(0).asFloat64())>1e-12;
    return false;
}

// Accept either explicit bool true or YARP vocab32('o','k') as success
static bool rpcAccepted(const yarp::os::Bottle& b) {
    if (b.size()==0) return false;
    if (b.get(0).isVocab32())
        return b.get(0).asVocab32() == yarp::os::createVocab32('o','k');
    return bottleAsBool(b);
}

// Flatten any Bottle into a vector<double> by unrolling sub-lists.
static void flattenBottleToDoubles(const yarp::os::Bottle& in, std::vector<double>& out) {
    out.clear();
    out.reserve(18);
    for (size_t i = 0; i < in.size(); ++i) {
        const auto& elem = in.get(i);
        if (elem.isList()) {
            yarp::os::Bottle* s = elem.asList();
            for (size_t j = 0; j < s->size(); ++j) {
                out.push_back(s->get(j).asFloat64());
            }
        } else {
            out.push_back(elem.asFloat64());
        }
    }
}

// ========================================
// Tunables: single-trajectory duration etc.
// ========================================
static constexpr double kTrajDurationSec = 15.0;   // one smooth move to target
static constexpr int    kPollMs          = 80;    // is_motion_done polling period
static constexpr int    kTimeoutMs       = 15000000; // overall wait timeout (ms)

// ==============================
// Lifecycle: start/close/spin
// ==============================
bool CartesianPointingComponent::start(int argc, char* argv[])
{
    // Initialize ROS2 only once (component may be created by an external launcher)
    if (!rclcpp::ok()) rclcpp::init(argc, argv);

    // We expect the cartesian controllers to be started externally (systemd / scripts)
    if (!yarp::os::Network::exists(cartesianPortLeft))
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Left Cartesian controller not present. Start it externally.");
    if (!yarp::os::Network::exists(cartesianPortRight))
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Right Cartesian controller not present. Start it externally.");

    // Wait until both RPC servers are up; we prefer a deterministic behavior here
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Waiting for LEFT controller port '%s'...", cartesianPortLeft.c_str());
    while (!yarp::os::Network::exists(cartesianPortLeft))
        std::this_thread::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Waiting for RIGHT controller port '%s'...", cartesianPortRight.c_str());
    while (!yarp::os::Network::exists(cartesianPortRight))
        std::this_thread::sleep_for(std::chrono::seconds(1));

    // Create the ROS2 node and a persistent TF buffer/listener
    m_node       = rclcpp::Node::make_shared("CartesianPointingComponentNode");
    m_tfBuffer   = std::make_unique<tf2_ros::Buffer>(m_node->get_clock());
    m_tfListener = std::make_unique<tf2_ros::TransformListener>(*m_tfBuffer);

    // Per-arm roll compensation (radians) around local Z of the EE frame.
    //  - LEFT defaults to +pi because left-hand meshes are often mirrored w.r.t. right.
    //  - RIGHT defaults to 0.
    double left_roll_bias  = M_PI;
    double right_roll_bias = 0.0;
    m_node->declare_parameter<double>("left_roll_bias_rad",  left_roll_bias);
    m_node->declare_parameter<double>("right_roll_bias_rad", right_roll_bias);
    // No need to read back here; defaults are declared and will be used unless overridden.

    // Open local YARP client ports and connect them to the remote servers
    if (yarp::os::Network::exists(m_cartesianPortNameLeft))
        yarp::os::Network::disconnect(m_cartesianPortNameLeft, cartesianPortLeft);
    if (!m_cartesianPortLeft.open(m_cartesianPortNameLeft)) {
        yError() << "Cannot open Left client port"; return false;
    }
    yarp::os::Network::connect(m_cartesianPortNameLeft, cartesianPortLeft);

    if (yarp::os::Network::exists(m_cartesianPortNameRight))
        yarp::os::Network::disconnect(m_cartesianPortNameRight, cartesianPortRight);
    if (!m_cartesianPortRight.open(m_cartesianPortNameRight)) {
        yError() << "Cannot open Right client port"; return false;
    }
    yarp::os::Network::connect(m_cartesianPortNameRight, cartesianPortRight);

    // ROS2 service (non-blocking handler: we spawn a thread and return immediately)
    m_srvPointAt = m_node->create_service<cartesian_pointing_interfaces::srv::PointAt>(
        "/CartesianPointingComponent/PointAt",
        [this](const std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Request> req,
               std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Response> res) {
            const std::string target = req->target_name;
            RCLCPP_INFO(m_node->get_logger(), "Service PointAt received: %s", target.c_str());
            std::thread([this, target]() {
                auto r = std::make_shared<cartesian_pointing_interfaces::srv::PointAt::Request>();
                r->target_name = target;
                this->pointTask(r);
            }).detach();
            res->is_ok = true; res->error_msg = ""; // service always acks immediately
        });

    // Map2D client device to lookup objects by name
    {
        yarp::os::Property p; p.put("device","map2D_nwc_yarp");
        p.put("local","/cartesian_pointing_component/map2d_nwc");
        p.put("remote","/map2D_nws_yarp");
        if (!m_map2dDevice.open(p)) {
            RCLCPP_ERROR(m_node->get_logger(),"Unable to open map2D_nwc_yarp"); return false;
        }
        m_map2dDevice.view(m_map2dView);
        if (!m_map2dView) {
            RCLCPP_ERROR(m_node->get_logger(),"Unable to obtain IMap2D view"); return false;
        }
        RCLCPP_INFO(m_node->get_logger(),"Device map2D_nwc_yarp opened and IMap2D view obtained");
    }

    // Optional: import POIs from a JSON for convenience during development
    importArtworksFromJson("/home/user1/UC3/yarp-contexts/contexts/r1_cartesian_control/artwork_coords.json");

    RCLCPP_DEBUG(m_node->get_logger(), "CartesianPointingComponent READY");
    return true;
}

bool CartesianPointingComponent::close() {
    if (m_cartesianPortLeft.isOpen())  m_cartesianPortLeft.close();
    if (m_cartesianPortRight.isOpen()) m_cartesianPortRight.close();
    rclcpp::shutdown();
    return true;
}

void CartesianPointingComponent::spin() { rclcpp::spin(m_node); }

// ==================
// TF2 helper methods
// ==================

bool CartesianPointingComponent::getTFMatrix(const std::string& target,const std::string& source,Eigen::Matrix4d& T) const {
    if (!m_tfBuffer) return false;
    try {
        // Lookup transform from 'source' to 'target' at latest time with a 1s timeout
        auto tf = m_tfBuffer->lookupTransform(target, source, rclcpp::Time(0), std::chrono::seconds(1));

        // Extract translation and rotation (geometry_msgs::msg::TransformStamped)
        const auto& tr = tf.transform.translation; 
        const auto& q  = tf.transform.rotation;

        // Convert to Eigen quaternion (note the (w,x,y,z) ordering expected by Eigen)
        Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);

        // Build homogeneous transform: rotation in top-left 3x3, translation in last column
        T.setIdentity();
        T.topLeftCorner<3,3>() = Q.toRotationMatrix();
        T(0,3)=tr.x; T(1,3)=tr.y; T(2,3)=tr.z;
        return true;
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN(m_node->get_logger(),"TF lookup failed %s<- %s : %s", target.c_str(), source.c_str(), e.what());
        return false;
    }
}

Eigen::Vector3d CartesianPointingComponent::transformPoint(const Eigen::Matrix4d& T,const Eigen::Vector3d& p) {
    Eigen::Vector4d ph; ph<<p,1.0; ph=T*ph; return ph.head<3>();
}

bool CartesianPointingComponent::transformPointMapToRobot(const geometry_msgs::msg::Point& map_p,
                                                          geometry_msgs::msg::Point& out_robot_p,
                                                          const std::string& robot_frame,double timeout_sec)
{
    if (!m_tfBuffer) { RCLCPP_WARN(m_node->get_logger(),"TF buffer not initialized"); return false; }
    const auto timeout = tf2::durationFromSec(timeout_sec);
    if (!m_tfBuffer->canTransform(robot_frame, m_mapFrame, tf2::TimePointZero, timeout)) {
        RCLCPP_WARN(m_node->get_logger(),"TF: transform %s <- %s not available", robot_frame.c_str(), m_mapFrame.c_str());
        return false;
    }
    try {
        auto tf = m_tfBuffer->lookupTransform(robot_frame, m_mapFrame, tf2::TimePointZero, timeout);
        geometry_msgs::msg::PointStamped in,out; in.header=tf.header; in.header.frame_id=m_mapFrame; in.point=map_p;
        tf2::doTransform(in,out,tf); out_robot_p=out.point; return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(m_node->get_logger(),"TF exception: %s", ex.what()); return false;
    }
}

// =============================================
// Map2D JSON loader (handy during development)
// =============================================
bool CartesianPointingComponent::importArtworksFromJson(const std::string& json_path)
{
    if (!m_map2dView) return false;
    std::ifstream ifs(json_path); if (!ifs.is_open()) return false;
    nlohmann::json j; try { ifs>>j; } catch (...) { return false; }
    if (!j.contains("artworks") || !j["artworks"].is_object()) return false;
    for (auto it=j["artworks"].begin(); it!=j["artworks"].end(); ++it) {
        yarp::dev::Nav2D::Map2DObject o; o.x=it.value()["x"]; o.y=it.value()["y"]; o.z=it.value()["z"];
        (void)m_map2dView->storeObject(it.key(), o);
    }
    std::vector<std::string> names; if (m_map2dView->getObjectsList(names)) {
        std::ostringstream oss; oss<<"Loaded Objects ("<<names.size()<<"):"; for (auto& n:names) oss<<" "<<n;
        RCLCPP_INFO(m_node->get_logger(), "%s", oss.str().c_str());
    }
    return true;
}

// ======================================================
// Wait for motion completion using `is_motion_done` RPC
// ======================================================
bool CartesianPointingComponent::waitMotionDone(yarp::os::Port* p, int poll_ms, int max_ms)
{
    if (!p || !p->isOpen()) return false;
    yarp::os::Bottle cmd,res; cmd.addString("is_motion_done");
    int waited=0; if (max_ms<0) max_ms=30000; // safety default
    while (true) {
        res.clear();
        if (!p->write(cmd,res)) return false;
        if (rpcAccepted(res) || bottleAsBool(res)) return true; // finished
        if (max_ms>0 && waited>=max_ms) return false;           // timeout
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
        waited+=poll_ms;
    }
}

// ====================================================================
// Build a “natural” pointing orientation (palm-down, Z towards target)
// ====================================================================
// We construct an orthonormal basis {x_ee, y_ee, z_ee}:
//   z_ee = -normalize(target - shoulder)
//          (minus because +Z in the mesh points *towards the robot*;
//           we want Z to look *outwards*, i.e., from hand to target)
//   y_ee = normalize( down - (down·z_ee) z_ee )
//          where down = (0,0,-1) in base frame → palm facing down
//   x_ee = y_ee × z_ee (right-handed frame)
static Eigen::Quaterniond buildPointingPalmDownQuat(const Eigen::Vector3d& shoulder_base,
                                                    const Eigen::Vector3d& target_base)
{
    // Direction shoulder → target
    Eigen::Vector3d dir = target_base - shoulder_base;
    const double n = dir.norm();
    if (n < 1e-9) return Eigen::Quaterniond::Identity(); // degenerate case
    dir /= n;

    // EE Z-axis: towards the target (accounting for the inward +Z convention)
    Eigen::Vector3d z_ee = -dir;

    // World/robot down direction used to make the palm face down
    const Eigen::Vector3d base_down(0.0, 0.0, -1.0);

    // Project base_down onto plane ⟂ z_ee to build EE Y-axis
    Eigen::Vector3d y_ee = base_down - (base_down.dot(z_ee))*z_ee;
    double ny = y_ee.norm();
    if (ny < 1e-6) {
        // If down is almost collinear with z_ee, pick any non-collinear backup,
        // then re-project and normalize (degenerate configuration)
        y_ee = (std::abs(z_ee.x()) < 0.9 ? Eigen::Vector3d(1,0,0) : Eigen::Vector3d(0,1,0));
        y_ee -= (y_ee.dot(z_ee))*z_ee;
        y_ee.normalize();
    } else {
        y_ee /= ny;
    }

    // EE X-axis from the right-handed cross product
    Eigen::Vector3d x_ee = y_ee.cross(z_ee);
    double nx = x_ee.norm();
    if (nx < 1e-6) {
        // Rare numerical fallback: use an orthogonal unit and recompute y
        x_ee = z_ee.unitOrthogonal();
        y_ee = z_ee.cross(x_ee).normalized();
    } else {
        x_ee /= nx;
    }

    // Rotation matrix and quaternion
    Eigen::Matrix3d R; R.col(0)=x_ee; R.col(1)=y_ee; R.col(2)=z_ee;
    Eigen::Quaterniond q(R); q.normalize();
    return q;
}

/**
 * @brief Per-arm local Z roll compensation (accounts for differences in hand meshes).
 *
 * Retrieves a roll bias (in radians) from ROS 2 parameters and builds a quaternion
 * representing a rotation about the local Z axis of the end-effector. This can be
 * used to slightly adjust the natural pointing orientation to match left/right hand
 * frame conventions.
 *
 * Parameters consulted:
 *  - left_roll_bias_rad  (default: +pi)
 *  - right_roll_bias_rad (default: 0)
 *
 * Typical composition: q_cmd = q_nat * q_fix, where q_nat is the analytically
 * computed pointing quaternion and q_fix is this compensation.
 *
 * @param armName Arm identifier ("LEFT" or "RIGHT").
 * @param node ROS 2 node used to read the parameters.
 * @return Eigen::Quaterniond The roll compensation quaternion. Identity if the
 *         resolved roll magnitude is near zero.
 */
static Eigen::Quaterniond armCompensationQuat(const std::string& armName,
                                              rclcpp::Node::SharedPtr node)
{
    double left_roll_bias= M_PI;   // default roll for LEFT
    double right_roll_bias=0.0;    // default roll for RIGHT
    (void)node->get_parameter("left_roll_bias_rad",  left_roll_bias);
    (void)node->get_parameter("right_roll_bias_rad", right_roll_bias);

    const double roll = (armName=="LEFT") ? left_roll_bias : right_roll_bias;
    if (std::abs(roll) < 1e-12) return Eigen::Quaterniond::Identity();
    return Eigen::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()));
}

// ==============================================================
// Pre-scan both arms to pick the most suited for the current task
// ==============================================================
bool CartesianPointingComponent::computeEndEffectorDistancesToTarget(
        const Eigen::Vector3d& artwork_pos,
        std::vector<std::pair<std::string,double>>& armDistances,
        std::map<std::string, std::vector<double>>& cachedPoseValues)
{
    armDistances.clear(); cachedPoseValues.clear();

    for (const std::string armId : {"LEFT","RIGHT"}) {
        yarp::os::Port* client = (armId=="LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
        const std::string server = (armId=="LEFT") ? cartesianPortLeft : cartesianPortRight;
        if (!client->isOpen() || !yarp::os::Network::isConnected(client->getName(), server)) {
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: %s port not ready", armId.c_str());
            continue;
        }

        // Query current EE pose primarily to estimate distance to target
        yarp::os::Bottle cmd,res; cmd.addString("get_pose");
        if (!client->write(cmd,res)) continue;

        std::vector<double> flat; flattenBottleToDoubles(res, flat);
        if (flat.size()!=16 && flat.size()!=18) continue; // unknown layout

        const size_t off = (flat.size()==18) ? 2u : 0u;  // some servers prepend 2 elements
        Eigen::Vector3d p_hand(flat[off+3], flat[off+7], flat[off+11]); // transl. entries
        armDistances.emplace_back(armId,(artwork_pos-p_hand).norm());
        cachedPoseValues.emplace(armId,std::move(flat));
    }
    return !armDistances.empty();
}

// Prefer LEFT if target is on +Y side in the torso frame, RIGHT otherwise
bool CartesianPointingComponent::chooseArmByTorsoY(const Eigen::Vector3d& p_base,std::string& outArm) const
{
    Eigen::Matrix4d T; if (!getTFMatrix(m_torsoFrame, m_baseFrame, T)) return false;
    Eigen::Vector3d p_torso = transformPoint(T, p_base);
    outArm = (p_torso.y() > 0.0) ? "LEFT" : "RIGHT"; return true;
}

// Very simple reach model: clip the requested point on a sphere centered at the shoulder.
Eigen::Vector3d CartesianPointingComponent::sphereReachPoint(const Eigen::Vector3d& shoulder_base,
                                                             const Eigen::Vector3d& target_base) const
{
    const double d = (target_base - shoulder_base).norm();
    const double R = std::min(m_reachRadius, std::max(d, m_minDist));
    if (d < 1e-6) return shoulder_base;  // avoid division by zero
    const Eigen::Vector3d dir = (target_base - shoulder_base)/d;
    const double L = std::min(d, R);     // clamp along the ray
    return shoulder_base + L * dir;
}

// ======================
// Main pointing routine
// ======================
void CartesianPointingComponent::pointTask(const std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Request> request)
{
    // 1) Lookup target coordinates by name from Map2D
    yarp::dev::Nav2D::Map2DObject obj; bool found=false;
    if (m_map2dView) found = m_map2dView->getObject(request->target_name, obj);
    if (!found) {
        RCLCPP_WARN(m_node->get_logger(),"Map2D: target '%s' not found", request->target_name.c_str());
        return;
    }

    // 2) Transform from map to base frame (if TF is available). If not, use raw coords.
    RCLCPP_INFO(m_node->get_logger(),"Target (map fram): x=%.3f y=%.3f z=%.3f", obj.x, obj.y, obj.z);
    Eigen::Vector3d p_base(obj.x,obj.y,obj.z);
    geometry_msgs::msg::Point mapPt, basePt; mapPt.x=obj.x; mapPt.y=obj.y; mapPt.z=obj.z;
    if (transformPointMapToRobot(mapPt, basePt, m_baseFrame, 1.0)) {
        p_base = {basePt.x, basePt.y, basePt.z};
        RCLCPP_INFO(m_node->get_logger(),"Target (base after TF): x=%.3f y=%.3f z=%.3f", basePt.x,basePt.y,basePt.z);
    } else {
        RCLCPP_WARN(m_node->get_logger(),"TF map->%s failed; using given coords as %s", m_baseFrame.c_str(), m_baseFrame.c_str());
        RCLCPP_INFO(m_node->get_logger(),"Target (base fallback): x=%.3f y=%.3f z=%.3f", p_base.x(),p_base.y(),p_base.z());
    }

    // 3) Pre-scan → pick one arm and freeze that choice (no later switching)
    std::vector<std::pair<std::string,double>> armDist;
    std::map<std::string,std::vector<double>> cached;
    if (!computeEndEffectorDistancesToTarget(p_base, armDist, cached)) {
        RCLCPP_ERROR(m_node->get_logger(),"No arms available"); return;
    }
    std::string pref; if (!chooseArmByTorsoY(p_base, pref)) pref="LEFT";
    std::stable_sort(armDist.begin(), armDist.end(), [&](auto&a,auto&b){
        if (a.first==pref && b.first!=pref) return true;
        if (a.first!=pref && b.first==pref) return false;
        return a.second < b.second;
    });
    armDist = { armDist.front() }; // keep only the best candidate

    { std::lock_guard<std::mutex> lk(m_flagMutex); m_isPointing = true; }

    for (const auto& armEntry : armDist) {
        const std::string arm = armEntry.first;
        yarp::os::Port* port = (arm=="LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
        const std::string remote = (arm=="LEFT") ? cartesianPortLeft : cartesianPortRight;
        if (!port->isOpen() || !yarp::os::Network::isConnected(port->getName(), remote)) {
            RCLCPP_WARN(m_node->get_logger(),"ARM %s: port not ready, abort", arm.c_str());
            break;
        }

        // 4) Read shoulder pose in base frame (needed both for position and orientation)
        const std::string shoulder_frame = (arm=="LEFT") ? m_lShoulderFrame : m_rShoulderFrame;
        Eigen::Matrix4d T_base_sh;
        if (!getTFMatrix(m_baseFrame, shoulder_frame, T_base_sh)) {
            RCLCPP_WARN(m_node->get_logger(),"ARM %s: missing shoulder TF, abort", arm.c_str());
            break;
        }
        const Eigen::Vector3d p_sh   = T_base_sh.block<3,1>(0,3);
        const Eigen::Vector3d p_goal = sphereReachPoint(p_sh, p_base); // clamp along reach sphere

        // 5) Build a palm-down quaternion that makes EE Z look toward the target
        Eigen::Quaterniond q_nat = buildPointingPalmDownQuat(p_sh, p_goal);

        // 6) Apply a per-arm roll compensation around local Z to account for hand frame differences
        Eigen::Quaterniond q_fix = armCompensationQuat(arm, m_node);
        Eigen::Quaterniond q_cmd = q_nat * q_fix;  // rotate in EE frame; preserves Z direction

        // 7) Issue a single `go_to_pose` command
        RCLCPP_INFO(m_node->get_logger(),
                    "ARM %s: go_to_pose target -> pos=(%.3f, %.3f, %.3f) quat=(%.4f, %.4f, %.4f, %.4f)",
                    arm.c_str(), p_goal.x(), p_goal.y(), p_goal.z(),
                    q_cmd.x(), q_cmd.y(), q_cmd.z(), q_cmd.w());

        // Ensure the controller state is Stop before sending a new trajectory
        { yarp::os::Bottle c,r; c.addString("stop"); (void)port->write(c,r); std::this_thread::sleep_for(std::chrono::milliseconds(100)); }

        yarp::os::Bottle cmd,res;
        cmd.addString("go_to_pose");
        cmd.addFloat64(p_goal.x()); cmd.addFloat64(p_goal.y()); cmd.addFloat64(p_goal.z());
        cmd.addFloat64(q_cmd.x());  cmd.addFloat64(q_cmd.y());  cmd.addFloat64(q_cmd.z()); cmd.addFloat64(q_cmd.w());
        cmd.addFloat64(kTrajDurationSec);

        const bool ok = port->write(cmd,res);
        if (!(ok && res.size()>0 && rpcAccepted(res))) {
            RCLCPP_WARN(m_node->get_logger(),"ARM %s: go_to_pose rejected, abort", arm.c_str());
            break; // do NOT switch arm
        }
        if (!waitMotionDone(port, kPollMs, kTimeoutMs)) {
            RCLCPP_WARN(m_node->get_logger(),"ARM %s: timeout waiting motion", arm.c_str());
            break; // do NOT switch arm
        }

        // 8) (Optional) Log final EE orientation and angular mismatch w.r.t. command
        yarp::os::Bottle cmd_get,res_get; cmd_get.addString("get_pose");
        if (port->write(cmd_get,res_get)) {
            std::vector<double> flat; flattenBottleToDoubles(res_get, flat);
            if (flat.size()==16 || flat.size()==18) {
                const size_t off=(flat.size()==18)?2u:0u;
                Eigen::Matrix3d R; R << flat[off+0],flat[off+1],flat[off+2],
                                        flat[off+4],flat[off+5],flat[off+6],
                                        flat[off+8],flat[off+9],flat[off+10];
                Eigen::Quaterniond qee(R); qee.normalize();
                auto quat_angle = [](const Eigen::Quaterniond& a, const Eigen::Quaterniond& b){
                    double d = std::abs(a.x()*b.x() + a.y()*b.y() + a.z()*b.z() + a.w()*b.w());
                    d = std::min(1.0, std::max(0.0, d));
                    return 2.0*std::acos(d);
                };
                const double ang = quat_angle(q_cmd, qee) * 180.0/M_PI;
                RCLCPP_INFO(m_node->get_logger(),
                            "ARM %s: final EE quat=(%.4f, %.4f, %.4f, %.4f)  [misalign=%.1f deg]",
                            arm.c_str(), qee.x(), qee.y(), qee.z(), qee.w(), ang);
            }
        }
        break; // success or handled failure → never switch arm
    }

    { std::lock_guard<std::mutex> lk(m_flagMutex); m_isPointing=false; }
}
