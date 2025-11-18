// =========================
// CartesianPointingComponent.cpp
// =========================
/**
 * @file CartesianPointingComponent.cpp
 * @brief Implementation of a ROS 2 / YARP component that points the robot arm to named targets.
 *
 * Responsibilities:
 *  - Initialize ROS 2 node, TF2 listener, YARP RPC clients, and the Map2D client
 *  - Expose a PointAt service that resolves a target name via IMap2D (Objects only; no fallback to Locations)
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
#include <cartesian_pointing_interfaces/srv/is_motion_done.hpp>
#include <bits/stdc++.h>

// ========================================
// Tunables: single trajectory and timings
// ========================================
/**
 * @brief Duration in seconds for a single smooth trajectory sent to the Cartesian controller.
 */
static constexpr double kTrajDurationSec = 1.0;
/**
 * @brief Polling period in milliseconds for the is_motion_done RPC.
 */
static constexpr int kPollMs = 80;
/**
 * @brief Overall timeout in milliseconds when waiting for motion completion.
 */
static constexpr int kTimeoutMs = 15000000;

/**
 * @brief Compatibility importer for Objects: parse the "Objects" section and push entries via IMap2D::storeObject().
 *
 * Supported syntaxes per line (orientation ignored):
 *  - name ( map x y z [roll pitch yaw "desc"] )
 *  - name map x y z [roll pitch yaw]
 * Only map id and x,y,z are parsed; any extra tokens (including roll/pitch/yaw/desc) are ignored.
 *
 * @param filePath Absolute path to the locations.ini file.
 * @param map2d IMap2D interface pointer where objects will be stored.
 * @param logger ROS logger for warnings/info.
 * @return size_t Number of objects successfully imported.
 */
 /*
static size_t importObjectsSectionWithIMap2D(
    const std::string &filePath,
    yarp::dev::Nav2D::IMap2D *map2d,
    rclcpp::Logger logger)
{
    // Safety check: if no IMap2D interface is provided, nothing can be imported
    if (!map2d)
        return 0;

    // Try to open the file
    std::ifstream f(filePath);
    if (!f.is_open())
    {
        RCLCPP_WARN(logger, "cannot open file: '%s'", filePath.c_str());
        return 0;
    }

    // Helper lambda: trims leading and trailing whitespace from a string
    auto trim = [](const std::string &s)
    {
        const char *ws = " \t\r\n";
        size_t a = s.find_first_not_of(ws), b = s.find_last_not_of(ws);
        if (a == std::string::npos)
            return std::string();
        return s.substr(a, b - a + 1);
    };

    bool inObjects = false; // whether we are currently inside the "Objects:" section
    size_t imported = 0;    // counter of imported objects
    std::string line;

    // Read file line by line
    while (std::getline(f, line))
    {
        std::string raw = line;
        line = trim(line);
        if (line.empty())
            continue; // skip empty lines

        // Wait until we encounter the "Objects:" header
        if (!inObjects)
        {
            if (line == "Objects:")
                inObjects = true;
            continue;
        }

        // The Objects section ends if we reach another header (Locations, Areas, Paths)
        if (line == "Locations:" || line == "Areas:" || line == "Paths:")
            break;

        // Skip comment lines
        if (line[0] == '#')
            continue;

        // Tokenize the line using stringstream
        std::istringstream iss(line);
        std::string name;
        if (!(iss >> name))
            continue; // if no first token (object name), skip

        // --------------------------------------------------------------------
        // a) Parenthesized form:
        //    name ( map x y z [roll pitch yaw "desc"] )
        // --------------------------------------------------------------------
        size_t parPos = raw.find('(');
        if (parPos != std::string::npos && parPos > raw.find(name))
        {
            // Find matching closing parenthesis
            size_t closePos = raw.find(')', parPos);
            if (closePos == std::string::npos)
            {
                RCLCPP_WARN(logger, "compat import: malformed line (missing ')'): %s", raw.c_str());
                continue;
            }

            // Extract content inside parentheses and trim it
            std::string inside = raw.substr(parPos + 1, closePos - parPos - 1);
            inside = trim(inside);

            // Tokenize the inside section: map x y z [ ...ignored... ]
            std::istringstream is2(inside);
            std::string map;
            double x = 0, y = 0, z = 0;
            if (!(is2 >> map >> x >> y >> z))
            {
                RCLCPP_WARN(logger, "compat import: cannot parse x y z: %s", raw.c_str());
                continue;
            }
            // Any additional tokens (including roll/pitch/yaw/"desc") are intentionally ignored.

            // Construct a Map2DObject and store it in the map
            yarp::dev::Nav2D::Map2DObject obj;
            obj.map_id = map;
            obj.x = x;
            obj.y = y;
            obj.z = z;
            obj.roll = 0;
            obj.pitch = 0;
            obj.yaw = 0; // orientation intentionally ignored
            obj.description = "";

            if (map2d->storeObject(name, obj))
            {
                ++imported;
            }
            else
            {
                RCLCPP_WARN(logger, "compat import: storeObject('%s') failed", name.c_str());
            }
            continue; // done with this line
        }

        // --------------------------------------------------------------------
        // b) Minimal form:
        //    name map x y z [ ...ignored... ]
        // --------------------------------------------------------------------
        {
            std::string map;
            double x = 0, y = 0, z = 0;
            if (!(iss >> map >> x >> y >> z))
            {
                RCLCPP_WARN(logger, "compat import: cannot parse minimal form: %s", raw.c_str());
                continue;
            }

            // Create and store the Map2DObject
            yarp::dev::Nav2D::Map2DObject obj;
            obj.map_id = map;
            obj.x = x;
            obj.y = y;
            obj.z = z;
            obj.roll = 0;
            obj.pitch = 0;
            obj.yaw = 0;
            obj.description = "";

            if (map2d->storeObject(name, obj))
            {
                ++imported;
            }
            else
            {
                RCLCPP_WARN(logger, "compat import: storeObject('%s') failed", name.c_str());
            }
        }
    }

    // Return total number of successfully imported objects
    return imported;
}*/

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
bool CartesianPointingComponent::start(int argc, char *argv[])
{
    if (!rclcpp::ok())
        rclcpp::init(argc, argv);

    // ROS 2 node and TF utilities first, so logging and params are available early
    m_node = rclcpp::Node::make_shared("CartesianPointingComponentNode");
    m_tfBuffer = std::make_unique<tf2_ros::Buffer>(m_node->get_clock());
    m_tfListener = std::make_unique<tf2_ros::TransformListener>(*m_tfBuffer);

    // YARP network required
    if (!yarp::os::Network::checkNetwork())
    {
        RCLCPP_ERROR(m_node->get_logger(), "YARP network is not available");
        return false;
    }

    // ROS 2 service
    m_srvPointAt = m_node->create_service<cartesian_pointing_interfaces::srv::PointAt>(
        "/CartesianPointingComponent/PointAt",
        [this](const std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Request> req,
               std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Response> res)
        {
            const std::string target = req->target_name;
            RCLCPP_INFO(m_node->get_logger(), "Service PointAt received: %s", target.c_str());
            std::thread([this, target]()
                        {
                auto r = std::make_shared<cartesian_pointing_interfaces::srv::PointAt::Request>();
                r->target_name = target;
                this->pointTask(r); })
                .detach();
            res->is_ok = true;
            res->error_msg = "";
        });

    // Status query service: /CartesianPointingComponent/IsMotionDone
    m_srvIsMotionDone = m_node->create_service<cartesian_pointing_interfaces::srv::IsMotionDone>(
        "/CartesianPointingComponent/IsMotionDone",
        [this](const std::shared_ptr<cartesian_pointing_interfaces::srv::IsMotionDone::Request>,
               std::shared_ptr<cartesian_pointing_interfaces::srv::IsMotionDone::Response> res)
        {
            std::lock_guard<std::mutex> lk(m_flagMutex);
            const bool done = !m_isPointing;
            res->is_done = done;
            res->is_ok = true;
        });

    // Map2D client
    {
        const std::string local = "/cartesian_pointing_component/map2d_nwc";
        const std::string remote = "/map2D_nws_yarp";

        RCLCPP_INFO(m_node->get_logger(), "Waiting for Map2D server '%s'...", remote.c_str());
        int wait_ms = 0;
        while (!yarp::os::Network::exists(remote) && wait_ms < 5000)
        { // wait up to 5s
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            wait_ms += 200;
        }
        if (!yarp::os::Network::exists(remote))
        {
            RCLCPP_WARN(m_node->get_logger(), "Map2D server '%s' not found, proceeding anyway", remote.c_str());
        }

        if (m_map2dClientDevice.isValid())
            m_map2dClientDevice.close();

        yarp::os::Property p;
        p.put("device", "map2D_nwc_yarp");
        p.put("local", local);
        p.put("remote", remote);

        if (!m_map2dClientDevice.open(p))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Unable to open map2D_nwc_yarp");
            return false;
        }
        if (!m_map2dClientDevice.isValid())
        {
            RCLCPP_ERROR(m_node->get_logger(), "map2D_nwc_yarp opened but device is not valid");
            m_map2dClientDevice.close();
            return false;
        }

        m_map2dClientDevice.view(m_map2dView);
        if (!m_map2dView)
        {
            RCLCPP_ERROR(m_node->get_logger(), "Unable to obtain IMap2D view");
            m_map2dClientDevice.close();
            return false;
        }
        RCLCPP_INFO(m_node->get_logger(), "Device map2D_nwc_yarp opened and IMap2D view obtained");
    }

    // Resolve locations.ini path: prefer argv[1], fallback to ROS param 'map2d_locations_file'
    /*std::string locations_file;
    std::string source_tag;
    if (argc >= 2 && argv && argv[1] && std::string(argv[1]).size() > 0)
    {
        locations_file = argv[1];
        source_tag = "argv[1]";
    }
    if (locations_file.empty())
    {
        locations_file = m_node->declare_parameter<std::string>("map2d_locations_file", "");
        if (!locations_file.empty())
            source_tag = "ROS param map2d_locations_file";
    }
    if (locations_file.empty())
    {
        RCLCPP_ERROR(m_node->get_logger(),
                     "No locations.ini provided. Pass it as first argument, or set ROS param 'map2d_locations_file'.");
        return false;
    }
    {
        std::ifstream test_f(locations_file);
        if (!test_f.is_open())
        {
            RCLCPP_ERROR(m_node->get_logger(),
                         "Locations file '%s' not found or not readable.", locations_file.c_str());
            return false;
        }
    }

    RCLCPP_INFO(m_node->get_logger(), "Using locations file '%s' (source: %s)", locations_file.c_str(), source_tag.c_str());

     bool ok = false;
    if (!ok)
    {
        const size_t n = importObjectsSectionWithIMap2D(locations_file, m_map2dView, m_node->get_logger());
        if (n > 0)
        {
            RCLCPP_INFO(m_node->get_logger(), "Imported %zu Objects from '%s' using IMap2D::storeObject()", n, locations_file.c_str());
            ok = true;
        }
        else
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to load any Objects from '%s'; aborting startup.", locations_file.c_str());
            return false;
        }
    }*/

    // Log available objects
    logAvailableObjects();

    // local temporary buffered port name (unique per arm)
    localJ = std::string("/cartesian_pointing_component/joints_reader_");

    actions = {
        {"h0v0",   0,  0}, {"h0v20",   0, 20}, {"h0v35",   0, 35},
        {"h30v0", 30,  0}, {"h30v20", 30, 20}, {"h30v35", 30, 35},
        {"h60v0", 60,  0}, {"h60v20", 60, 20}, {"h60v35", 60, 35},
        {"h90v0", 90,  0}, {"h90v20", 90, 20}, {"h90v35", 90, 35}
    };

    RCLCPP_DEBUG(m_node->get_logger(), "CartesianPointingComponent READY");
    return true;
}

/**
 * @brief Gracefully close YARP ports and shutdown ROS 2.
 * @return true on success.
 */
bool CartesianPointingComponent::close()
{
    if (m_cartesianControlServerLeftPort.isOpen())
        m_cartesianControlServerLeftPort.close();
    if (m_cartesianControlServerRightPort.isOpen())
        m_cartesianControlServerRightPort.close();
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
bool CartesianPointingComponent::getTFMatrix(const std::string &target, const std::string &source, Eigen::Matrix4d &T) const
{
    if (!m_tfBuffer)
        return false;
    try
    {
        auto tf = m_tfBuffer->lookupTransform(target, source, rclcpp::Time(0), std::chrono::seconds(1));
        const auto &tr = tf.transform.translation;
        const auto &q = tf.transform.rotation;
        Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);
        T.setIdentity();
        T.topLeftCorner<3, 3>() = Q.toRotationMatrix();
        T(0, 3) = tr.x;
        T(1, 3) = tr.y;
        T(2, 3) = tr.z;
        return true;
    }
    catch (const tf2::TransformException &e)
    {
        RCLCPP_WARN(m_node->get_logger(), "TF lookup failed %s<- %s : %s", target.c_str(), source.c_str(), e.what());
        return false;
    }
}

/**
 * @brief Apply a homogeneous transform to a 3D point.
 * @param T 4x4 homogeneous transform.
 * @param p 3D point to transform.
 * @return Transformed 3D point.
 */
Eigen::Vector3d CartesianPointingComponent::transformPoint(const Eigen::Matrix4d &T, const Eigen::Vector3d &p)
{
    Eigen::Vector4d ph;
    ph << p, 1.0;
    ph = T * ph;
    return ph.head<3>();
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
bool CartesianPointingComponent::transformPointMapToRobot(const geometry_msgs::msg::Point &map_p,
                                                          geometry_msgs::msg::Point &out_robot_p,
                                                          const std::string &source_frame,
                                                          const std::string &robot_frame,
                                                          double timeout_sec)
{
    if (!m_tfBuffer)
    {
        RCLCPP_WARN(m_node->get_logger(), "TF buffer not initialized");
        return false;
    }
    const auto timeout = tf2::durationFromSec(timeout_sec);
    if (!m_tfBuffer->canTransform(robot_frame, source_frame, tf2::TimePointZero, timeout))
    {
        RCLCPP_WARN(m_node->get_logger(), "TF: transform %s <- %s not available", robot_frame.c_str(), source_frame.c_str());
        return false;
    }
    try
    {
        auto tf = m_tfBuffer->lookupTransform(robot_frame, source_frame, tf2::TimePointZero, timeout);
        geometry_msgs::msg::PointStamped in, out;
        in.header = tf.header;
        in.header.frame_id = source_frame;
        in.point = map_p;
        tf2::doTransform(in, out, tf);
        out_robot_p = out.point;
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN(m_node->get_logger(), "TF exception: %s", ex.what());
        return false;
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
    if (!m_map2dView)
    {
        RCLCPP_WARN(m_node->get_logger(), "IMap2D view not available; cannot list objects");
        return;
    }
    // Prefer retrieving names to print "name and coordinates"
    std::vector<std::string> objNames;
    if (m_map2dView->getObjectsList(objNames))
    {
        RCLCPP_INFO(m_node->get_logger(), "Available objects (%zu)", objNames.size());
        if (objNames.empty())
        {
            RCLCPP_INFO(m_node->get_logger(), "Hint: check locations.ini or compat-import.");
        }
        for (const auto &name : objNames)
        {
            yarp::dev::Nav2D::Map2DObject o;
            if (m_map2dView->getObject(name, o))
            {
                // Print: object name, coordinates, and orientation (rpy in radians; not used)
                RCLCPP_INFO(m_node->get_logger(),
                            " - %s: pos=(%.3f, %.3f, %.3f) rpy(rad)=(%.3f, %.3f, %.3f)",
                            name.c_str(), o.x, o.y, o.z, o.roll, o.pitch, o.yaw);
            }
        }
        return;
    }

    // Fallback: print without names if only the full objects list is available
    std::vector<yarp::dev::Nav2D::Map2DObject> allObjs;
    if (m_map2dView->getAllObjects(allObjs))
    {
        RCLCPP_INFO(m_node->get_logger(), "Available objects (%zu)", allObjs.size());
        if (allObjs.empty())
        {
            RCLCPP_INFO(m_node->get_logger(), "Hint: check locations.ini or compat-import.");
        }
        for (const auto &o : allObjs)
        {
            RCLCPP_INFO(m_node->get_logger(), " - x=%.3f y=%.3f z=%.3f",
                        o.x, o.y, o.z);
        }
        return;
    }

    RCLCPP_WARN(m_node->get_logger(), "IMap2D: unable to retrieve objects list");
}

// ==============================================================
// Simple reach model and arm selection
// ==============================================================
/**
 * @brief Compute a reachable point along the line shoulder → target, clipped by a spherical reach.
 * @param shoulder_base Shoulder position in base frame.
 * @param target_base Target position in base frame.
 * @return Clipped reachable point in base frame.
 */
Eigen::Vector3d CartesianPointingComponent::sphereReachPoint(const Eigen::Vector3d &shoulder_base,
                                                             const Eigen::Vector3d &target_base) const
{
    const double d = (target_base - shoulder_base).norm();
    const double R = std::min(m_reachRadius, std::max(d, m_minDist));
    if (d < 1e-6)
        return shoulder_base;
    const Eigen::Vector3d dir = (target_base - shoulder_base) / d;
    const double L = std::min(d, R);
    return shoulder_base + L * dir;
}

/**
 * @brief Heuristic to select left/right arm by inspecting the target Y coordinate in the torso frame.
 * @param p_base Target position in base frame.
 * @param[out] outArm Set to "LEFT" if target.y > 0 in torso frame, "RIGHT" otherwise.
 * @return true if TF was available to compute the torso transform; false otherwise.
 */
bool CartesianPointingComponent::chooseArmByTorsoY(const Eigen::Vector3d &p_base, std::string &outArm) const
{
    Eigen::Matrix4d T;
    if (!getTFMatrix(m_torsoFrame, m_baseFrame, T))
        return false;
    Eigen::Vector3d p_torso = transformPoint(T, p_base);
    outArm = (p_torso.y() > 0.0) ? "LEFT" : "RIGHT";
    return true;
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

/**
 * @brief Build a pointing orientation (hand Z toward target, palm down, lateral bias by arm preference).
 *
 * Given the shoulder position, the target position (both in base frame), and a lateral preference ("LEFT"/"RIGHT"),
 * this function computes a hand orientation quaternion that:
 *   1) Aligns the hand's +Z axis *back* toward the robot (i.e., -dir from shoulder to target),
 *      so the finger tips point *outward* toward the target.
 *   2) Tries to keep the palm (Y axis) facing downward (aligned with base -Z).
 *   3) Respects the lateral preference: for "RIGHT", prefer the hand X axis aligned with base +Y (left);
 *      for "LEFT", prefer X aligned with base -Y (robot's right), yielding a more natural left-arm gesture.
 *
 * Implementation steps:
 *   - Compute z_ee = -(target - shoulder).normalized() (pointing direction, inverted).
 *   - Project base_down onto the plane perpendicular to z_ee to get candidate y_ee directions.
 *   - Generate multiple candidate orientations (y and -y for robustness in degenerate configurations).
 *   - Score each candidate by:
 *       a) y·base_down > 0  (palm facing down),
 *       b) lateral preference check on the resulting x (x·base_left sign matches the preference).
 *   - Return the best candidate, ensuring orthogonal, right-handed basis (X, Y, Z) → quaternion.
 *
 * @param shoulder_base  3D position of the shoulder in base frame.
 * @param target_base    3D position of the pointing target in base frame.
 * @param pref           Lateral preference: "LEFT" or "RIGHT".
 * @return               Quaternion representing the desired hand orientation (identity if dir too small).
 */


// ======================================================
/**
 * @brief Convert a 1-element Bottle to bool, supporting bool/int/double conventions.
 * @param b Input Bottle.
 * @return true if Bottle converts to a non-zero/true value; false otherwise.
 */
static bool bottleAsBool(const yarp::os::Bottle &b)
{
    if (b.size() == 0)
        return false;
    if (b.get(0).isBool())
        return b.get(0).asBool();
    if (b.get(0).isInt32())
        return b.get(0).asInt32() != 0;
    if (b.get(0).isFloat64())
        return std::abs(b.get(0).asFloat64()) > 1e-12;
    return false;
}
/**
 * @brief Interpret an RPC reply as acceptance: either boolean true or vocab32('o','k').
 * @param b Input Bottle.
 * @return true if the reply is accepted; false otherwise.
 */
static bool rpcAccepted(const yarp::os::Bottle &b)
{
    if (b.size() == 0)
        return false;
    if (b.get(0).isVocab32())
        return b.get(0).asVocab32() == yarp::os::createVocab32('o', 'k');
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

// ======================
// Pointing routine
// ======================
/**
 * @brief Core pointing task executed asynchronously upon PointAt service requests.
 *
 * Flow:
 *  1) Lookup target coordinates by name from Map2D (Objects only)
 *  2) Transform target from map to base frame (if TF available)
 *  3) Choose a single arm (no switching) using torso-Y heuristic
 *  4) Compute a reachable goal and a natural orientation (palm-down, Z to target) with per-arm roll bias
 *  5) Send one go_to_pose and wait for completion
 *
 * @param request Service request containing the target name.
 */
void CartesianPointingComponent::pointTask(const std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Request> request)
{
    // 1) Try as OBJECT (3D x,y,z); if not present, abort (no fallback to Locations)
    double map_x = 0, map_y = 0, map_z = 0;
    std::string map_frame;
    bool found = false;

    if (m_map2dView)
    {
        yarp::dev::Nav2D::Map2DObject obj;
        if (m_map2dView->getObject(request->target_name, obj))
        {
            map_x = obj.x;
            map_y = obj.y;
            map_z = obj.z;
            map_frame = "map";
            found = true;
            RCLCPP_INFO(m_node->get_logger(), "Target '%s' as OBJECT in map '%s': x=%.3f y=%.3f z=%.3f",
                        request->target_name.c_str(), map_frame.c_str(), map_x, map_y, map_z);
        }
    }
    if (!found)
    {
        RCLCPP_ERROR(m_node->get_logger(), "Map2D: target '%s' not found as Object. Aborting.",
                     request->target_name.c_str());
        return;
    }

    // 2) Transform to base frame
    Eigen::Vector3d p_base(map_x, map_y, map_z);
    geometry_msgs::msg::Point mapPt, basePt;
    mapPt.x = map_x;
    mapPt.y = map_y;
    mapPt.z = map_z;
    if (!map_frame.empty() && transformPointMapToRobot(mapPt, basePt, map_frame, m_baseFrame, 1.0))
    {
        p_base = {basePt.x, basePt.y, basePt.z};
        RCLCPP_INFO(m_node->get_logger(), "Target (base after TF): x=%.3f y=%.3f z=%.3f", basePt.x, basePt.y, basePt.z);
    }
    else
    {
        RCLCPP_WARN(m_node->get_logger(), "TF %s->%s unavailable; using map coords as base",
                    map_frame.c_str(), m_baseFrame.c_str());
        RCLCPP_INFO(m_node->get_logger(), "Target (base fallback): x=%.3f y=%.3f z=%.3f", p_base.x(), p_base.y(), p_base.z());
    }

    // 3) Choose arm and reach the point (orientation computed as "palm-down", independent of objects)
    std::vector<std::pair<std::string, double>> armDist;
    // For simplicity we skip pre-scan here; choose by torso Y sign for quick consistency:
    std::string pref;
    if (!chooseArmByTorsoY(p_base, pref))
        pref = "LEFT";
    armDist = {{pref, 0.0}};

    {
        std::lock_guard<std::mutex> lk(m_flagMutex);
        m_isPointing = true;
    }
    bool behind = false;
    for (const auto &armEntry : armDist)
    {
        const std::string arm = armEntry.first;

        const std::string shoulder_frame = (arm == "LEFT") ? m_lShoulderFrame : m_rShoulderFrame;
        Eigen::Matrix4d T_base_sh;
        if (!getTFMatrix(m_baseFrame, shoulder_frame, T_base_sh))
        {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: missing shoulder TF, abort", arm.c_str());
            break;
        }
        const Eigen::Vector3d p_sh = T_base_sh.block<3, 1>(0, 3);
        const Eigen::Vector3d p_goal = sphereReachPoint(p_sh, p_base);

        if (p_goal.x() < 0)
        {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: goal behind robot (x=%.3f), abort", arm.c_str(), p_goal.x());
            behind = true;
            break;
        }

        // Check: Euclidean distance from origin (sqrt(x^2+y^2+z^2)) must be >= 40
        
        const double dist = std::sqrt(p_goal.x() * p_goal.x() + p_goal.y() * p_goal.y() + p_goal.z() * p_goal.z());
        if (dist < 0.40)
        {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: goal too close (dist=%.3f < 40.0), abort", arm.c_str(), dist);
            break;
        }

        RCLCPP_INFO(m_node->get_logger(),
                    "ARM %s: go_to_pose -> pos=(%.3f, %.3f, %.3f)",
                    arm.c_str(), p_goal.x(), p_goal.y(), p_goal.z());
        yInfo() << "p_goal.z() - 0.80: " << p_goal.z() - 0.80;

        double theta_x = computeThetaX(p_goal.x(), p_goal.y());
        double theta_z = computeThetaZ(p_goal.x(), p_goal.z()-0.80);

        yInfo() << "Theta_x: " << theta_x << ", Theta_z: " << theta_z;

        // Find the best action along h and select the candidates
        double bestH = 10000.0;
        std::vector<int> cand;
        for (int i = 0; i < (int)actions.size(); ++i) {
            double dh = angularDist(theta_x, actions[i].h, 0.0);
            yInfo() << "Action: " << actions[i].name << ", dh: " << dh;
            if (dh < bestH - 1e-12) {       
                bestH = dh;
                cand.clear();
                cand.push_back(i);
            } else if (fabs(dh - bestH) <= 1e-12) {
                cand.push_back(i);
            }
        }

        // Among candidates, find the best along v
        double bestV = 10000.0;
        int bestIdx = -1;
        for (int i : cand) {
            double dv = angularDist(theta_z, actions[i].v, 0.0);
            if (dv < bestV - 1e-12) {
                bestV = dv;
                bestIdx = i;
            }
        }
        bestPointing = actions[bestIdx].name;
        yInfo() << "Action scelta: " << bestPointing
            << " (|dh|=" << bestH << ", |dv|=" << bestV << ")\n";


    }

    {
        std::lock_guard<std::mutex> lk(m_flagMutex);
        m_isPointing = false;
    }
    if (!behind)
    {
        ExecuteDance(pref);
    }
    

}

double CartesianPointingComponent::angularDist(double a, double b, double period) {
    a = abs(a);
    yInfo() << "Angular Dist called with a: " << a << ", b: " << b << ", period: " << period;
    if (period <= 0.0) return fabs(a - b);
    double d = fmod(fabs(a - b), period);
    return std::min(d, period - d);
}


void CartesianPointingComponent::ExecuteDance(std::string pointingArm)
{
    // ---------------------------------Execute Dance Component Service ExecuteDance------------------------------
    yInfo() << "[CartesianPointingComponent::ExecuteDance] Starting Execute Dance Service";
    auto executeDanceClientNode = rclcpp::Node::make_shared("CartesianPointingComponentExecuteDanceNode");

    auto danceClient = executeDanceClientNode->create_client<execute_dance_interfaces::srv::ExecuteDance>("/ExecuteDanceComponent/ExecuteDance");
    auto dance_request = std::make_shared<execute_dance_interfaces::srv::ExecuteDance::Request>();

    if (pointingArm == "LEFT")
    {
        dance_request->dance_name = "left_" + bestPointing;
    }
    else if (pointingArm == "RIGHT")
    {
        dance_request->dance_name = "right_" + bestPointing;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid pointing arm specified for ExecuteDance: %s", pointingArm.c_str());
        return;
    }

    // Wait for service
    while (!danceClient->wait_for_service(std::chrono::milliseconds(100)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'ExecuteDance'. Exiting.");
        }
    }
    auto dance_result = danceClient->async_send_request(dance_request);

    if (rclcpp::spin_until_future_complete(executeDanceClientNode, dance_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execute Dance succeeded");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service execute_dance");
        return;
    }
}

double CartesianPointingComponent::computeThetaX(double x, double y)
{
    double theta_rad = atan2(y,x);
    double theta_deg = theta_rad*180/M_PI;
    return theta_deg;
}

double CartesianPointingComponent::computeThetaZ(double x, double z)
{
    double theta_rad = atan2(z,x);
    double theta_deg = theta_rad*180/M_PI;
    return theta_deg;
}

