// =========================
// CartesianPointingComponent.h
// =========================
#pragma once

/**
 * @file CartesianPointingComponent.h
 * @brief Declarations for a ROS 2 / YARP component that points the robot arm to named targets.
 *
 * This component:
 *  - Connects to two Cartesian controllers (left/right arm) via YARP RPC
 *  - Exposes a ROS 2 service `PointAt` to point an arm towards a named target
 *  - Queries a Map2D server to fetch target 3D coordinates by name
 *  - Computes a single end-effector pose (position + orientation) and sends one `go_to_pose`
 *  - Waits for motion completion by polling `is_motion_done`
 *
 * Design choices:
 *  - We avoid a two-stage motion (position then orientation). The end-effector pose is
 *    computed analytically from current shoulder position and the target, then sent once.
 *  - Only one arm is selected (no mid-execution switching) to avoid inconsistent behaviors.
 *  - Orientation convention (for natural pointing):
 *      - The model's +Z axis of the hand frame points inwards to the robot.
 *      - To make the finger/palm face the target, we point the EE Z-axis towards the target:
 *        z_ee = -dir(shoulder → target).
 *      - We want the palm facing down (EE Y-axis pointing downwards). Build y_ee by projecting
 *        the world-down vector (0,0,-1) onto the plane orthogonal to z_ee, then x_ee = y_ee × z_ee.
 *        This yields a right-handed rotation matrix R = [x_ee y_ee z_ee].
 *  - A per-arm roll compensation around local Z can be applied via ROS parameters to match
 *    hand frame conventions between left/right.
 */

#include <mutex>
#include <thread>
#include <memory>
#include <map>
#include <vector>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/Map2DObject.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cartesian_pointing_interfaces/srv/point_at.hpp>

class CartesianPointingComponent
{
public:
    CartesianPointingComponent() = default;

    /**
     * @brief Initialize ROS 2 node, TF listeners, YARP ports, Map2D client, and the PointAt service.
     *
     * This will also wait for the remote Cartesian controller RPC ports to be available and connect
     * the local client ports.
     *
     * @param[in] argc Main argc (forwarded to rclcpp if ROS 2 wasn't initialized).
     * @param[in] argv Main argv (forwarded to rclcpp if ROS 2 wasn't initialized).
     * @return true if initialization completed successfully; false otherwise.
     */
    bool start(int argc, char* argv[]);

    /**
     * @brief Gracefully close YARP ports and shutdown ROS 2.
     * @return true on success.
     */
    bool close();

    /**
     * @brief Spin the ROS 2 node (blocking call).
     */
    void spin();

private:
    /**
     * @brief Core task executed on PointAt service requests.
     *
     * Steps:
     *  1) Lookup target coordinates by name from Map2D
     *  2) Transform target from map to base frame (if TF available)
     *  3) Pre-scan and choose a single arm to use
     *  4) Compute a reachable goal point along shoulder → target
     *  5) Build a natural pointing orientation (palm-down, Z towards target)
     *  6) Apply optional per-arm roll compensation
     *  7) Send one `go_to_pose` and wait for completion
     *
     * @param[in] request Service request containing the target name.
     */
    void pointTask(const std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Request> request);

    /**
     * @brief Query current EE pose for each arm and compute distance to the target.
     *
     * Used only to decide which arm to use; it does not command any motion.
     *
     * @param[in] artwork_pos Target position in base frame (meters).
     * @param[out] armDistances Vector of pairs {armId, distance} where armId ∈ {"LEFT","RIGHT"}.
     * @return true if at least one arm responded; false otherwise.
     */
    bool computeEndEffectorDistancesToTarget(const Eigen::Vector3d& artwork_pos,
                                             std::vector<std::pair<std::string,double>>& armDistances);

    /**
     * @brief Transform a point from a specified source frame into a robot frame using TF2.
     *
     * @param[in] map_point Point in the source frame.
     * @param[out] out_robot_point Transformed point in the destination robot frame.
     * @param[in] source_frame Source frame id (e.g., object.map_id).
     * @param[in] robot_frame Destination frame id (e.g., base frame).
     * @param[in] timeout_sec TF lookup timeout in seconds.
     * @return true if the transform was available and applied; false otherwise.
     */
    bool transformPointMapToRobot(const geometry_msgs::msg::Point& map_point,
                                  geometry_msgs::msg::Point& out_robot_point,
                                  const std::string& source_frame,
                                  const std::string& robot_frame,
                                  double timeout_sec);

    /**
     * @brief Retrieve a 4x4 homogeneous transform T(target ← source) from TF.
     *
     * The matrix maps points expressed in the source frame into the target frame:
     * [p_target;1] = T * [p_source;1].
     *
     * @param[in] target Destination frame id.
     * @param[in] source Origin frame id.
     * @param[out] T Output homogeneous transform.
     * @return true on success; false on TF unavailability or lookup failure.
     */
    bool getTFMatrix(const std::string& target, const std::string& source, Eigen::Matrix4d& T) const;

    /**
     * @brief Apply a homogeneous transform to a 3D point.
     * @param[in] T 4x4 homogeneous transform.
     * @param[in] p 3D point.
     * @return Transformed 3D point.
     */
    static Eigen::Vector3d transformPoint(const Eigen::Matrix4d& T, const Eigen::Vector3d& p);

    /**
     * @brief Heuristic to select left/right arm by inspecting the target Y coordinate in the torso frame.
     * @param[in] p_base Target position in base frame.
     * @param[out] outArm Set to "LEFT" if target.y > 0 in torso frame, "RIGHT" otherwise.
     * @return true if TF was available to compute the torso transform; false otherwise.
     */
    bool chooseArmByTorsoY(const Eigen::Vector3d& p_base, std::string& outArm) const;

    /**
     * @brief Compute a reachable point along the line shoulder → target, clipped by a spherical reach.
     * @param[in] shoulder_base Shoulder position in base frame.
     * @param[in] target_base Target position in base frame.
     * @return Clipped reachable point in base frame.
     */
    Eigen::Vector3d sphereReachPoint(const Eigen::Vector3d& shoulder_base, const Eigen::Vector3d& target_base) const;

    // (Removed) JSON import helper; locations are now obtained directly from IMap2D

    /**
     * @brief Poll the controller with `is_motion_done` until motion ends or a timeout occurs.
     *
     * Accepts either a boolean true or the YARP vocab32('o','k') as completion.
     *
     * @param[in] p Open YARP port connected to the controller RPC server.
     * @param[in] poll_ms Polling period in milliseconds.
     * @param[in] max_ms Maximum wait time in milliseconds (-1 to use a safety default).
     * @return true if motion completed; false on timeout or communication error.
     */
    bool waitMotionDone(yarp::os::Port* p, int poll_ms = 80, int max_ms = -1);

    /**
     * @brief Log all available objects from the IMap2D server.
     *
     * Queries the list of stored objects and prints both the list of names
     * and the detailed info (x, y, z) for each one. Intended to be called at
     * startup for visibility/debugging.
     */
    void logAvailableObjects();

private:
    // ROS2 core
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<cartesian_pointing_interfaces::srv::PointAt>::SharedPtr m_srvPointAt;

    /// TF2 utilities (persistent buffer + listener)
    std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tfListener;

    /// Frame names (tune if your robot uses different naming)
    std::string m_baseFrame      {"mobile_base_body_link"};
    std::string m_torsoFrame     {"chest_link"};
    std::string m_lShoulderFrame {"l_shoulder_1"};
    std::string m_rShoulderFrame {"r_shoulder_1"};

    /// Very light reach model: maximum radius from shoulder and a minimum stand-off
    double m_reachRadius   {0.60};
    double m_minDist       {0.10};

    /// YARP cartesian controller server ports (must be running externally)
    const std::string cartesianPortLeft  = "/cartesian-control/left_arm/rpc:i";
    const std::string cartesianPortRight = "/cartesian-control/right_arm/rpc:i";

    /// Local client ports (this component talks through these)
    std::string m_cartesianPortNameLeft  = "/CartesianPointingComponent/cartesianClientLeft/rpc";
    std::string m_cartesianPortNameRight = "/CartesianPointingComponent/cartesianClientRight/rpc";
    yarp::os::Port m_cartesianPortLeft;
    yarp::os::Port m_cartesianPortRight;

    /// Map2D client device & view
    yarp::dev::PolyDriver m_map2dDevice;
    yarp::dev::Nav2D::IMap2D* m_map2dView = nullptr; // interface pointer

    /// Runtime flag: true while a pointing routine is active
    std::mutex m_flagMutex;
    bool m_isPointing {false};
};