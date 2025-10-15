#pragma once
// CartesianPointingComponent (header)
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
// #include <cartesian_pointing_interfaces/srv/is_pointing.hpp> // removed

class CartesianPointingComponent
{
public:
    CartesianPointingComponent() = default;

    // Initialize ROS2, TF, YARP ports and services
    bool start(int argc, char* argv[]);
    bool close();
    void spin();

private:
    void pointTask(const std::shared_ptr<cartesian_pointing_interfaces::srv::PointAt::Request> request);
    bool preScanArticulatedArms(const Eigen::Vector3d& artwork_pos,std::vector<std::pair<std::string,double>>& armDistances,std::map<std::string, std::vector<double>>& cachedPoseValues);
    bool isPoseReachable(yarp::os::Port* activePort,const Eigen::Vector3d& candidate,const Eigen::Quaterniond& q_target);
    bool transformPointMapToRobot(const geometry_msgs::msg::Point& map_point,geometry_msgs::msg::Point& out_robot_point,const std::string& robot_frame,double timeout_sec);
    bool getTFMatrix(const std::string& target,const std::string& source,Eigen::Matrix4d& T) const;
    static Eigen::Vector3d transformPoint(const Eigen::Matrix4d& T,const Eigen::Vector3d& p);
    // Decide preferred arm based on torso Y and other heuristics
    bool chooseArmByTorsoY(const Eigen::Vector3d& p_base,std::string& outArm) const;
    // Get shoulder position expressed in base frame
    bool getShoulderPosInBaseFrame(const std::string& arm,Eigen::Vector3d& out_pos) const;
    // Compute candidate point on the shoulder->target line clipped by reach sphere
    Eigen::Vector3d sphereReachPoint(const Eigen::Vector3d& shoulder_base, const Eigen::Vector3d& target_base) const;
    bool importArtworksFromJson(const std::string& json_path); // -> TO BE REMOVED WHEN LOADING FROM MAP SERVER

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<cartesian_pointing_interfaces::srv::PointAt>::SharedPtr   m_srvPointAt;
    std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tfListener;
    std::string m_mapFrame        {"map"};
    std::string m_baseFrame       {"mobile_base_body_link"};
    std::string m_torsoFrame      {"chest_link"};
    std::string m_lShoulderFrame  {"l_shoulder_1"};
    std::string m_rShoulderFrame  {"r_shoulder_1"};
    double m_reachRadius   {0.60};
    double m_minDist       {0.40};
    double m_safetyBackoff {0.05};
    // YARP cartesian controller server ports
    const std::string cartesianPortLeft  = "/cartesian-control/left_arm/rpc:i";
    const std::string cartesianPortRight = "/cartesian-control/right_arm/rpc:i";

    // Local client port names opened by this component
    std::string m_cartesianPortNameLeft  = "/CartesianPointingComponent/cartesianClientLeft/rpc";
    std::string m_cartesianPortNameRight = "/CartesianPointingComponent/cartesianClientRight/rpc";
    yarp::os::Port m_cartesianPortLeft;
    yarp::os::Port m_cartesianPortRight;

    // NWC device for Map2DObject: manages the connection to the server
    yarp::dev::PolyDriver m_map2dDevice;
    // Pointer to the interface to query 2D objects
    yarp::dev::Nav2D::IMap2D* m_map2dView = nullptr; // interface pointer
    // Runtime flags and synchronization
    std::mutex m_flagMutex;
    bool m_isPointing {false};
};
