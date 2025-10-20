// =========================
// CartesianPointingComponent.cpp
// =========================
/**
 * @file CartesianPointingComponent.cpp
 * @brief Implementation of a ROS 2 / YARP component that points the robot arm to named targets.
 *
 * Responsibilities:
 *  - Initialize ROS 2 node, TF2 listener, YARP RPC clients, and the Map2D client
 *  - Expose a PointAt service that resolves a target name via IMap2D (Objects preferred, fallback to Locations)
 *  - Transform target coordinates into the robot base frame
 *  - Select an arm and compute a natural end-effector orientation (palm-down, Z towards target)
 *  - Send a single go_to_pose and wait for completion using the controller's RPC interface
 *
 * Notes:
 *  - This file contains convenience helpers (bottle parsing, quaternion utilities, etc.).
 *  - Public API and main class documentation live in the header file.
 */
#include "CartesianPointingComponent.h"

#include <sstream>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <string>

// ========================================
// Tunables: single trajectory and timings
// ========================================
/**
 * @brief Duration in seconds for a single smooth trajectory sent to the Cartesian controller.
 */
static constexpr double kTrajDurationSec = 15.0;
/**
 * @brief Polling period in milliseconds for the is_motion_done RPC.
 */
static constexpr int    kPollMs          = 80;
/**
 * @brief Overall timeout in milliseconds when waiting for motion completion.
 */
static constexpr int    kTimeoutMs       = 15000000;

// =====================================================
// Helper: detect "Version:" (1,2,3,...) from the .ini file
// (diagnostic logging only; does not block anything)
// =====================================================
/**
 * @brief Detect the numeric value that follows the tag "Version:" in a locations.ini file.
 *
 * Diagnostic helper: used only to log the detected format version; it does not block startup.
 *
 * @param path Absolute path to the locations.ini file.
 * @param[out] why Optional pointer to a string receiving a human-readable error reason when detection fails.
 * @return int Detected version (>=0) on success; -1 on failure.
 */
static int detectLocationsFileVersion(const std::string& path, std::string* why=nullptr)
{
    std::ifstream f(path);
    if (!f.is_open()) { if (why) *why = "cannot open file"; return -1; }

    auto trim = [](std::string s){
        const char* ws = " \t\r\n";
        size_t a = s.find_first_not_of(ws);
        size_t b = s.find_last_not_of(ws);
        if (a==std::string::npos) return std::string();
        return s.substr(a, b-a+1);
    };

    std::string line; bool sawVersionTag=false;
    while (std::getline(f,line)) {
        std::string raw=line; line=trim(line);
        if (line.empty()) continue;
        std::string l=line; for (auto& c:l) c=std::tolower(c);

        if (!sawVersionTag) {
            if (l=="version:" || l=="version") { sawVersionTag=true; continue; }
            if (l.rfind("version:",0)==0) {
                std::string rhs = trim(raw.substr(std::string("Version:").size()));
                if (!rhs.empty()) { try { return std::stoi(rhs); } catch (...) { if (why) *why="invalid version value"; return -1; } }
                sawVersionTag=true; continue;
            }
            continue;
        } else {
            try { return std::stoi(line); } catch (...) { if (why) *why="invalid version value"; return -1; }
        }
    }
    if (why) *why = sawVersionTag ? "missing numeric value after Version:" : "Version: tag not found";
    return -1;
}

// ========================================
// Compatibility parser for the Objects section:
// - supports the extended form:
//     name ( map x y z roll pitch yaw "desc" )
// - and the minimal form (only x y z):
//     name map x y z
// where roll/pitch/yaw/desc are optional.
// Objects are inserted ONLY via IMap2D::storeObject.
// ========================================
/**
 * @brief Compatibility importer for Objects: parse the "Objects" section and push entries via IMap2D::storeObject().
 *
 * Supported syntaxes per line:
 *  - name ( map x y z roll pitch yaw "desc" )
 *  - name map x y z [roll pitch yaw]
 * Unrecognized fields are ignored. Description is ignored as well.
 *
 * @param filePath Absolute path to the locations.ini file.
 * @param map2d IMap2D interface pointer where objects will be stored.
 * @param logger ROS logger for warnings/info.
 * @return size_t Number of objects successfully imported.
 */
static size_t importObjectsSectionWithIMap2D(const std::string& filePath,
                                             yarp::dev::Nav2D::IMap2D* map2d,
                                             rclcpp::Logger logger)
{
    if (!map2d) return 0;
    std::ifstream f(filePath);
    if (!f.is_open()) {
        RCLCPP_WARN(logger, "compat import: cannot open '%s'", filePath.c_str());
        return 0;
    }

    auto trim = [](const std::string& s){
        const char* ws=" \t\r\n";
        size_t a=s.find_first_not_of(ws), b=s.find_last_not_of(ws);
        if (a==std::string::npos) return std::string();
        return s.substr(a, b-a+1);
    };

    bool inObjects=false; size_t imported=0;
    std::string line;
    while (std::getline(f,line)) {
        std::string raw=line; line=trim(line);
        if (line.empty()) continue;

        if (!inObjects) {
            if (line=="Objects:" || line=="Objects") inObjects=true;
            continue;
        }
        // Objects section ends when another header is encountered
        if (line=="Locations:" || line=="Areas:" || line=="Paths:") break;
        if (line[0]=='#') continue; // comments

        // Try parsing
        std::istringstream iss(line);
        std::string name; if (!(iss >> name)) continue;

        // a) parenthesized form: name ( map x y z roll pitch yaw "desc" )
        size_t parPos = raw.find('(');
        if (parPos!=std::string::npos && parPos > raw.find(name)) {
            // extract content within parentheses
            size_t closePos = raw.find(')', parPos);
            if (closePos==std::string::npos) {
                RCLCPP_WARN(logger, "compat import: malformed line (missing ')'): %s", raw.c_str());
                continue;
            }
            std::string inside = raw.substr(parPos+1, closePos-parPos-1);
            inside = trim(inside);
            // tokenize: map x y z [roll pitch yaw "desc"]
            std::istringstream is2(inside);
            std::string map; double x=0,y=0,z=0, roll=0,pitch=0,yaw=0;
            if (!(is2 >> map >> x >> y >> z)) {
                RCLCPP_WARN(logger, "compat import: cannot parse x y z: %s", raw.c_str());
                continue;
            }
            // optional values
            (void)(is2 >> roll >> pitch >> yaw); // if absent they remain 0

            // description (optional, double-quoted) – not needed here; deliberately ignored

            yarp::dev::Nav2D::Map2DObject obj;
            obj.map_id = map; obj.x=x; obj.y=y; obj.z=z; obj.roll=0; obj.pitch=0; obj.yaw=0; obj.description="";
            if (map2d->storeObject(name, obj)) {
                ++imported;
            } else {
                RCLCPP_WARN(logger, "compat import: storeObject('%s') failed", name.c_str());
            }
            continue;
        }

        // b) minimal form: name map x y z [optional...]
        {
            std::string map; double x=0,y=0,z=0;
            if (!(iss >> map >> x >> y >> z)) {
                RCLCPP_WARN(logger, "compat import: cannot parse minimal form: %s", raw.c_str());
                continue;
            }
            yarp::dev::Nav2D::Map2DObject obj;
            obj.map_id = map; obj.x=x; obj.y=y; obj.z=z; obj.roll=0; obj.pitch=0; obj.yaw=0; obj.description="";
            if (map2d->storeObject(name, obj)) {
                ++imported;
            } else {
                RCLCPP_WARN(logger, "compat import: storeObject('%s') failed", name.c_str());
            }
        }
    }

    return imported;
}

// ==============================
// Lifecycle: start/close/spin
// ==============================
/**
 * @brief Initialize ROS 2, TF, YARP controllers clients, Map2D client and register the PointAt service.
 *
 * Waits for external Cartesian controller RPC ports to be available and connects to them.
 * Optionally attempts to load locations.ini via IMap2D if a file path is provided.
 *
 * @param argc Forwarded to rclcpp::init if ROS 2 is not yet initialized.
 * @param argv Forwarded to rclcpp::init if ROS 2 is not yet initialized.
 * @return true on success; false on initialization failures.
 */
bool CartesianPointingComponent::start(int argc, char* argv[])
{
    if (!rclcpp::ok()) rclcpp::init(argc, argv);

    // Warn if Cartesian controllers are not available (does not block startup)
    if (!yarp::os::Network::exists(cartesianPortLeft))
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Left Cartesian controller not present. Start it externally.");
    if (!yarp::os::Network::exists(cartesianPortRight))
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Right Cartesian controller not present. Start it externally.");

    // Deterministically wait for both controllers to be available
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Waiting for LEFT controller port '%s'...", cartesianPortLeft.c_str());
    while (!yarp::os::Network::exists(cartesianPortLeft))
        std::this_thread::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Waiting for RIGHT controller port '%s'...", cartesianPortRight.c_str());
    while (!yarp::os::Network::exists(cartesianPortRight))
        std::this_thread::sleep_for(std::chrono::seconds(1));

    // ROS 2 node and TF utilities
    m_node       = rclcpp::Node::make_shared("CartesianPointingComponentNode");
    m_tfBuffer   = std::make_unique<tf2_ros::Buffer>(m_node->get_clock());
    m_tfListener = std::make_unique<tf2_ros::TransformListener>(*m_tfBuffer);

    // Per-arm roll bias (does not affect objects – orientation is independent of targets)
    double left_roll_bias  = M_PI;
    double right_roll_bias = 0.0;
    m_node->declare_parameter<double>("left_roll_bias_rad",  left_roll_bias);
    m_node->declare_parameter<double>("right_roll_bias_rad", right_roll_bias);

    // YARP client ports towards the Cartesian controllers
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

    // ROS 2 service
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
            res->is_ok = true; res->error_msg = "";
        });

    // Map2D client
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

    // Resolve locations.ini path
    std::string default_locations_file = "";
    std::string source = "none";
    if (const char* env = std::getenv("MAP2D_LOCATIONS_FILE")) {
        default_locations_file = std::string(env);
        source = "env MAP2D_LOCATIONS_FILE";
        RCLCPP_INFO(m_node->get_logger(),"Default map2d_locations_file from %s: %s", source.c_str(), default_locations_file.c_str());
    }
    if (default_locations_file.empty()) {
        const std::string fallback_path = "/usr/local/src/robot/tour-guide-robot/app/maps/locations.ini";
        std::ifstream f(fallback_path);
        if (f.good()) { default_locations_file = fallback_path; source = "fallback"; }
    }
    m_node->declare_parameter<std::string>("map2d_locations_file", default_locations_file);
    std::string locations_file = "";
    if (m_node->get_parameter("map2d_locations_file", locations_file) && !locations_file.empty())
        source = "ROS param map2d_locations_file";

    if (!locations_file.empty()) {
        RCLCPP_INFO(m_node->get_logger(),"Using locations file '%s' (source: %s)", locations_file.c_str(), source.c_str());

        std::string why; int ver = detectLocationsFileVersion(locations_file, &why);
        if (ver >= 0) {
            RCLCPP_INFO(m_node->get_logger(),"Detected locations.ini version: %d (path: %s)", ver, locations_file.c_str());
        } else {
            RCLCPP_WARN(m_node->get_logger(),"Could not detect Version in '%s': %s", locations_file.c_str(), why.c_str());
        }

    // 1) Standard attempt via the IMap2D interface
        bool ok = static_cast<bool>(m_map2dView->loadLocationsAndExtras(locations_file));
        RCLCPP_INFO(m_node->get_logger(),"IMap2D.loadLocationsAndExtras('%s') -> %s",
                    locations_file.c_str(), ok ? "ok" : "fail");

    // 2) If it fails, try a compatibility import ONLY for Objects using storeObject
        if (!ok) {
            const size_t n = importObjectsSectionWithIMap2D(locations_file, m_map2dView, m_node->get_logger());
            if (n > 0) {
                RCLCPP_INFO(m_node->get_logger(),"Compat-imported %zu Objects from '%s' using IMap2D::storeObject()", n, locations_file.c_str());
            } else {
                RCLCPP_WARN(m_node->get_logger(),"No Objects compat-imported from '%s'", locations_file.c_str());
            }
        }
    } else {
        RCLCPP_INFO(m_node->get_logger(),
                    "No locations.ini provided. Set ROS param 'map2d_locations_file' or env MAP2D_LOCATIONS_FILE to load objects at startup.");
    }

    // Log available objects
    logAvailableObjects();

    RCLCPP_DEBUG(m_node->get_logger(), "CartesianPointingComponent READY");
    return true;
}

/**
 * @brief Gracefully close YARP ports and shutdown ROS 2.
 * @return true on success.
 */
bool CartesianPointingComponent::close() {
    if (m_cartesianPortLeft.isOpen())  m_cartesianPortLeft.close();
    if (m_cartesianPortRight.isOpen()) m_cartesianPortRight.close();
    rclcpp::shutdown();
    return true;
}

/**
 * @brief Spin the ROS 2 node (blocking).
 */
void CartesianPointingComponent::spin() { rclcpp::spin(m_node); }

// ==================
// TF2 helper methods
// ==================
/**
 * @brief Retrieve a 4x4 homogeneous transform T(target ← source) from TF.
 *
 * The matrix maps points expressed in the source frame into the target frame:
 * [p_target;1] = T * [p_source;1].
 *
 * @param target Destination frame id.
 * @param source Origin frame id.
 * @param[out] T Output homogeneous transform.
 * @return true on success; false on TF unavailability or lookup failure.
 */
bool CartesianPointingComponent::getTFMatrix(const std::string& target,const std::string& source,Eigen::Matrix4d& T) const {
    if (!m_tfBuffer) return false;
    try {
        auto tf = m_tfBuffer->lookupTransform(target, source, rclcpp::Time(0), std::chrono::seconds(1));
        const auto& tr = tf.transform.translation;
        const auto& q  = tf.transform.rotation;
        Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);
        T.setIdentity();
        T.topLeftCorner<3,3>() = Q.toRotationMatrix();
        T(0,3)=tr.x; T(1,3)=tr.y; T(2,3)=tr.z;
        return true;
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN(m_node->get_logger(),"TF lookup failed %s<- %s : %s", target.c_str(), source.c_str(), e.what());
        return false;
    }
}

/**
 * @brief Apply a homogeneous transform to a 3D point.
 * @param T 4x4 homogeneous transform.
 * @param p 3D point to transform.
 * @return Transformed 3D point.
 */
Eigen::Vector3d CartesianPointingComponent::transformPoint(const Eigen::Matrix4d& T,const Eigen::Vector3d& p) {
    Eigen::Vector4d ph; ph<<p,1.0; ph=T*ph; return ph.head<3>();
}

/**
 * @brief Transform a point from a specified source frame into a robot frame using TF2.
 *
 * @param map_p Point in the source frame.
 * @param[out] out_robot_p Transformed point in the destination robot frame.
 * @param source_frame Source frame id (e.g., object.map_id).
 * @param robot_frame Destination frame id (e.g., base frame).
 * @param timeout_sec TF lookup timeout in seconds.
 * @return true if the transform was available and applied; false otherwise.
 */
bool CartesianPointingComponent::transformPointMapToRobot(const geometry_msgs::msg::Point& map_p,
                                                          geometry_msgs::msg::Point& out_robot_p,
                                                          const std::string& source_frame,
                                                          const std::string& robot_frame,
                                                          double timeout_sec)
{
    if (!m_tfBuffer) { RCLCPP_WARN(m_node->get_logger(),"TF buffer not initialized"); return false; }
    const auto timeout = tf2::durationFromSec(timeout_sec);
    if (!m_tfBuffer->canTransform(robot_frame, source_frame, tf2::TimePointZero, timeout)) {
        RCLCPP_WARN(m_node->get_logger(),"TF: transform %s <- %s not available", robot_frame.c_str(), source_frame.c_str());
        return false;
    }
    try {
        auto tf = m_tfBuffer->lookupTransform(robot_frame, source_frame, tf2::TimePointZero, timeout);
        geometry_msgs::msg::PointStamped in,out; in.header=tf.header; in.header.frame_id=source_frame; in.point=map_p;
        tf2::doTransform(in,out,tf); out_robot_p=out.point; return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(m_node->get_logger(),"TF exception: %s", ex.what()); return false;
    }
}

// ==================
// Log objects
// ==================
/**
 * @brief Log all available objects from the IMap2D server.
 *
 * Queries and prints detailed information for each stored object.
 */
void CartesianPointingComponent::logAvailableObjects()
{
    if (!m_map2dView) {
        RCLCPP_WARN(m_node->get_logger(), "IMap2D view not available; cannot list objects");
        return;
    }
    std::vector<yarp::dev::Nav2D::Map2DObject> allObjs;
    if (m_map2dView->getAllObjects(allObjs)) {
        RCLCPP_INFO(m_node->get_logger(),"Available objects (%zu)", allObjs.size());
        if (allObjs.empty()) {
            RCLCPP_INFO(m_node->get_logger(),"Hint: check locations.ini or compat-import.");
        }
        for (const auto& o : allObjs) {
            RCLCPP_INFO(m_node->get_logger(),
                        " - map='%s' x=%.3f y=%.3f z=%.3f  desc='%s'",
                        o.map_id.c_str(), o.x, o.y, o.z, o.description.c_str());
        }
    } else {
        std::vector<std::string> objNames;
        if (!m_map2dView->getObjectsList(objNames)) {
            RCLCPP_WARN(m_node->get_logger(), "IMap2D: unable to retrieve objects list");
            return;
        }
        RCLCPP_INFO(m_node->get_logger(),"Available objects (%zu):", objNames.size());
        for (const auto& name : objNames) {
            yarp::dev::Nav2D::Map2DObject o;
            if (m_map2dView->getObject(name, o)) {
                RCLCPP_INFO(m_node->get_logger(),
                            " - %s: map='%s' x=%.3f y=%.3f z=%.3f",
                            name.c_str(), o.map_id.c_str(), o.x, o.y, o.z);
            }
        }
    }
}

// ==============================================================
// Simple reach model and arm selection (as in your version)
// ==============================================================
/**
 * @brief Compute a reachable point along the line shoulder → target, clipped by a spherical reach.
 * @param shoulder_base Shoulder position in base frame.
 * @param target_base Target position in base frame.
 * @return Clipped reachable point in base frame.
 */
Eigen::Vector3d CartesianPointingComponent::sphereReachPoint(const Eigen::Vector3d& shoulder_base,
                                                             const Eigen::Vector3d& target_base) const
{
    const double d = (target_base - shoulder_base).norm();
    const double R = std::min(m_reachRadius, std::max(d, m_minDist));
    if (d < 1e-6) return shoulder_base;
    const Eigen::Vector3d dir = (target_base - shoulder_base)/d;
    const double L = std::min(d, R);
    return shoulder_base + L * dir;
}

/**
 * @brief Heuristic to select left/right arm by inspecting the target Y coordinate in the torso frame.
 * @param p_base Target position in base frame.
 * @param[out] outArm Set to "LEFT" if target.y > 0 in torso frame, "RIGHT" otherwise.
 * @return true if TF was available to compute the torso transform; false otherwise.
 */
bool CartesianPointingComponent::chooseArmByTorsoY(const Eigen::Vector3d& p_base,std::string& outArm) const
{
    Eigen::Matrix4d T; if (!getTFMatrix(m_torsoFrame, m_baseFrame, T)) return false;
    Eigen::Vector3d p_torso = transformPoint(T, p_base);
    outArm = (p_torso.y() > 0.0) ? "LEFT" : "RIGHT"; return true;
}

/**
 * @brief Build a natural pointing orientation (palm-down, z-axis towards the target).
 *
 * Constructs an orthonormal basis {x_ee, y_ee, z_ee} such that:
 *  - z_ee points from end-effector towards the target (account for hand meshes with +Z inward)
 *  - y_ee aligns with world "down" projected on the plane orthogonal to z_ee (palm-down)
 *  - x_ee = y_ee × z_ee (right-handed)
 *
 * @param shoulder_base Shoulder position in base frame.
 * @param target_base Target position in base frame (possibly clipped elsewhere).
 * @return Unit quaternion representing the desired EE orientation.
 */
static Eigen::Quaterniond buildPointingPalmDownQuat(const Eigen::Vector3d& shoulder_base,
                                                    const Eigen::Vector3d& target_base)
{
    Eigen::Vector3d dir = target_base - shoulder_base;
    const double n = dir.norm();
    if (n < 1e-9) return Eigen::Quaterniond::Identity();
    dir /= n;
    Eigen::Vector3d z_ee = -dir;
    const Eigen::Vector3d base_down(0.0, 0.0, -1.0);
    Eigen::Vector3d y_ee = base_down - (base_down.dot(z_ee))*z_ee;
    double ny = y_ee.norm();
    if (ny < 1e-6) {
        y_ee = (std::abs(z_ee.x()) < 0.9 ? Eigen::Vector3d(1,0,0) : Eigen::Vector3d(0,1,0));
        y_ee -= (y_ee.dot(z_ee))*z_ee;
        y_ee.normalize();
    } else { y_ee /= ny; }
    Eigen::Vector3d x_ee = y_ee.cross(z_ee);
    double nx = x_ee.norm();
    if (nx < 1e-6) { x_ee = z_ee.unitOrthogonal(); y_ee = z_ee.cross(x_ee).normalized(); }
    else { x_ee /= nx; }
    Eigen::Matrix3d R; R.col(0)=x_ee; R.col(1)=y_ee; R.col(2)=z_ee;
    Eigen::Quaterniond q(R); q.normalize(); return q;
}

/**
 * @brief Per-arm local Z roll compensation quaternion.
 *
 * Reads ROS 2 parameters:
 *  - left_roll_bias_rad  (default: +pi)
 *  - right_roll_bias_rad (default: 0)
 * Compose as q_cmd = q_nat * q_fix to preserve the z_ee pointing direction.
 *
 * @param armName Arm identifier ("LEFT" or "RIGHT").
 * @param node ROS 2 node used to access parameters.
 * @return Unit quaternion for the roll compensation (Identity if near-zero).
 */
static Eigen::Quaterniond armCompensationQuat(const std::string& armName,
                                              rclcpp::Node::SharedPtr node)
{
    double left_roll_bias= M_PI;
    double right_roll_bias=0.0;
    (void)node->get_parameter("left_roll_bias_rad",  left_roll_bias);
    (void)node->get_parameter("right_roll_bias_rad", right_roll_bias);
    const double roll = (armName=="LEFT") ? left_roll_bias : right_roll_bias;
    if (std::abs(roll) < 1e-12) return Eigen::Quaterniond::Identity();
    return Eigen::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()));
}

// ======================================================
// Wait for motion completion (as in previous version)
// ======================================================
/**
 * @brief Convert a 1-element Bottle to bool, supporting bool/int/double conventions.
 * @param b Input Bottle.
 * @return true if Bottle converts to a non-zero/true value; false otherwise.
 */
static bool bottleAsBool(const yarp::os::Bottle& b) {
    if (b.size()==0) return false;
    if (b.get(0).isBool())    return b.get(0).asBool();
    if (b.get(0).isInt32())   return b.get(0).asInt32()!=0;
    if (b.get(0).isFloat64()) return std::abs(b.get(0).asFloat64())>1e-12;
    return false;
}
/**
 * @brief Interpret an RPC reply as acceptance: either boolean true or vocab32('o','k').
 * @param b Input Bottle.
 * @return true if the reply is accepted; false otherwise.
 */
static bool rpcAccepted(const yarp::os::Bottle& b) {
    if (b.size()==0) return false;
    if (b.get(0).isVocab32())
        return b.get(0).asVocab32() == yarp::os::createVocab32('o','k');
    return bottleAsBool(b);
}
/**
 * @brief Poll the controller with is_motion_done until motion ends or a timeout occurs.
 *
 * Accepts either a boolean true or the YARP vocab32('o','k') as completion.
 *
 * @param p Open YARP port connected to the controller RPC server.
 * @param poll_ms Polling period in milliseconds.
 * @param max_ms Maximum wait time in milliseconds (-1 to use a safety default).
 * @return true if motion completed; false on timeout or communication error.
 */
bool CartesianPointingComponent::waitMotionDone(yarp::os::Port* p, int poll_ms, int max_ms)
{
    if (!p || !p->isOpen()) return false;
    yarp::os::Bottle cmd,res; cmd.addString("is_motion_done");
    int waited=0; if (max_ms<0) max_ms=30000;
    while (true) {
        res.clear();
        if (!p->write(cmd,res)) return false;
        if (rpcAccepted(res) || bottleAsBool(res)) return true;
        if (max_ms>0 && waited>=max_ms) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
        waited+=poll_ms;
    }
}

// ======================
// Pointing routine
// ======================
/**
 * @brief Core pointing task executed asynchronously upon PointAt service requests.
 *
 * Flow:
 *  1) Lookup target coordinates by name from Map2D (Object first, fallback to Location+default Z)
 *  2) Transform target from map to base frame (if TF available)
 *  3) Choose a single arm (no switching) using torso-Y heuristic
 *  4) Compute a reachable goal and a natural orientation (palm-down, Z to target) with per-arm roll bias
 *  5) Send one go_to_pose and wait for completion
 *
 * @param request Service request containing the target name.
 */
void CartesianPointingComponent::pointTask(const std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Request> request)
{
    // 1) Try as OBJECT (3D x,y,z); if not present, fallback to LOCATION (2D) + default z
    double map_x=0, map_y=0, map_z=0;
    std::string map_frame;
    bool found=false;

    if (m_map2dView) {
        yarp::dev::Nav2D::Map2DObject obj;
        if (m_map2dView->getObject(request->target_name, obj)) {
            map_x=obj.x; map_y=obj.y; map_z=obj.z; map_frame=obj.map_id; found=true;
            RCLCPP_INFO(m_node->get_logger(),"Target '%s' as OBJECT in map '%s': x=%.3f y=%.3f z=%.3f",
                        request->target_name.c_str(), map_frame.c_str(), map_x, map_y, map_z);
        }
    }
    if (!found) {
        yarp::dev::Nav2D::Map2DLocation loc;
        if (!m_map2dView || !m_map2dView->getLocation(request->target_name, loc)) {
            RCLCPP_WARN(m_node->get_logger(),"Map2D: target '%s' not found as Object or Location",
                        request->target_name.c_str());
            return;
        }
        double default_z = 1.2; (void)m_node->get_parameter("location_default_z", default_z);
        map_x=loc.x; map_y=loc.y; map_z=default_z; map_frame=loc.map_id;
        RCLCPP_INFO(m_node->get_logger(),
                    "Target '%s' as LOCATION in map '%s': x=%.3f y=%.3f (z=default %.3f)",
                    request->target_name.c_str(), map_frame.c_str(), map_x, map_y, map_z);
    }

    // 2) Transform to base frame
    Eigen::Vector3d p_base(map_x,map_y,map_z);
    geometry_msgs::msg::Point mapPt, basePt; mapPt.x=map_x; mapPt.y=map_y; mapPt.z=map_z;
    if (!map_frame.empty() && transformPointMapToRobot(mapPt, basePt, map_frame, m_baseFrame, 1.0)) {
        p_base = {basePt.x, basePt.y, basePt.z};
        RCLCPP_INFO(m_node->get_logger(),"Target (base after TF): x=%.3f y=%.3f z=%.3f", basePt.x, basePt.y, basePt.z);
    } else {
        RCLCPP_WARN(m_node->get_logger(),"TF %s->%s unavailable; using map coords as base",
                    map_frame.c_str(), m_baseFrame.c_str());
        RCLCPP_INFO(m_node->get_logger(),"Target (base fallback): x=%.3f y=%.3f z=%.3f", p_base.x(), p_base.y(), p_base.z());
    }

    // 3) Choose arm and reach the point (orientation computed as "palm-down", independent of objects)
    std::vector<std::pair<std::string,double>> armDist;
    // For simplicity we skip pre-scan here; choose by torso Y sign for quick consistency:
    std::string pref; if (!chooseArmByTorsoY(p_base, pref)) pref="LEFT";
    armDist = { {pref, 0.0} };

    { std::lock_guard<std::mutex> lk(m_flagMutex); m_isPointing = true; }

    for (const auto& armEntry : armDist) {
        const std::string arm = armEntry.first;
        yarp::os::Port* port = (arm=="LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
        const std::string remote = (arm=="LEFT") ? cartesianPortLeft : cartesianPortRight;
        if (!port->isOpen() || !yarp::os::Network::isConnected(port->getName(), remote)) {
            RCLCPP_WARN(m_node->get_logger(),"ARM %s: port not ready, abort", arm.c_str());
            break;
        }

        const std::string shoulder_frame = (arm=="LEFT") ? m_lShoulderFrame : m_rShoulderFrame;
        Eigen::Matrix4d T_base_sh;
        if (!getTFMatrix(m_baseFrame, shoulder_frame, T_base_sh)) {
            RCLCPP_WARN(m_node->get_logger(),"ARM %s: missing shoulder TF, abort", arm.c_str());
            break;
        }
        const Eigen::Vector3d p_sh   = T_base_sh.block<3,1>(0,3);
        const Eigen::Vector3d p_goal = sphereReachPoint(p_sh, p_base);

        Eigen::Quaterniond q_nat = buildPointingPalmDownQuat(p_sh, p_goal);
        Eigen::Quaterniond q_fix = armCompensationQuat(arm, m_node);
        Eigen::Quaterniond q_cmd = q_nat * q_fix;

        RCLCPP_INFO(m_node->get_logger(),
                    "ARM %s: go_to_pose -> pos=(%.3f, %.3f, %.3f) quat=(%.4f, %.4f, %.4f, %.4f)",
                    arm.c_str(), p_goal.x(), p_goal.y(), p_goal.z(),
                    q_cmd.x(), q_cmd.y(), q_cmd.z(), q_cmd.w());

        { yarp::os::Bottle c,r; c.addString("stop"); (void)port->write(c,r); std::this_thread::sleep_for(std::chrono::milliseconds(100)); }

        yarp::os::Bottle cmd,res;
        cmd.addString("go_to_pose");
        cmd.addFloat64(p_goal.x()); cmd.addFloat64(p_goal.y()); cmd.addFloat64(p_goal.z());
        cmd.addFloat64(q_cmd.x());  cmd.addFloat64(q_cmd.y());  cmd.addFloat64(q_cmd.z()); cmd.addFloat64(q_cmd.w());
        cmd.addFloat64(kTrajDurationSec);

        const bool ok = port->write(cmd,res);
        if (!(ok && res.size()>0 && (res.get(0).asVocab32()==yarp::os::createVocab32('o','k') || bottleAsBool(res)))) {
            RCLCPP_WARN(m_node->get_logger(),"ARM %s: go_to_pose rejected, abort", arm.c_str());
            break;
        }
        if (!waitMotionDone(port, kPollMs, kTimeoutMs)) {
            RCLCPP_WARN(m_node->get_logger(),"ARM %s: timeout waiting motion", arm.c_str());
            break;
        }
        break; // niente switch di braccio
    }

    { std::lock_guard<std::mutex> lk(m_flagMutex); m_isPointing=false; }
}