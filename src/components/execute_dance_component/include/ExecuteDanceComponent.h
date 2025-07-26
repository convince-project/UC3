/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
#pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <dance_interfaces/srv/get_dance.hpp>
#include <dance_interfaces/srv/get_dance_duration.hpp>
#include <dance_interfaces/srv/get_movement.hpp>
#include <dance_interfaces/srv/set_dance.hpp>
#include <dance_interfaces/srv/get_part_names.hpp>
#include <dance_interfaces/srv/update_movement.hpp>
#include <execute_dance_interfaces/srv/execute_dance.hpp>
#include <execute_dance_interfaces/srv/is_dancing.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <map>

/**
 * @class ExecuteDanceComponent
 * @brief Main component for executing dance movements and pointing gestures on UC3 robot
 * 
 * This component manages the execution of dance movements for tour guiding activities,
 * including pointing gestures towards artworks in museums.
 * It integrates ROS2 services with YARP controllers for robot motion control.
 * 
 * Key features:
 * - Dance movement execution via YARP CTP services
 * - Pointing gestures towards specific artworks using Cartesian control
 * - Real-time robot localization integration
 * - Thread-safe movement queue management
 */
class ExecuteDanceComponent 
{
public:
    /**
     * @brief Default constructor
     */
    ExecuteDanceComponent() = default;
    
    /**
     * @brief Initialize and start the component
     * @param argc Command line argument count
     * @param argv Command line arguments
     * @return true if initialization successful, false otherwise
     */
    bool start(int argc, char*argv[]);
    
    /**
     * @brief Close and cleanup the component
     * @return true if cleanup successful, false otherwise
     */
    bool close();
    
    /**
     * @brief Main execution loop for ROS2 node spinning
     */
    void spin();
    
    // ========== ROS2 Service Handlers ==========
    
    /**
     * @brief Handle execute dance service requests
     * @param request Execute dance request containing dance name and parameters
     * @param response Execute dance response with execution status
     */
    void ExecuteDance(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request,
                      std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Response> response);
    
    /**
     * @brief Handle is dancing status requests
     * @param request Is dancing request
     * @param response Is dancing response with current dancing status
     */
    void IsDancing(const std::shared_ptr<execute_dance_interfaces::srv::IsDancing::Request> request,
                   std::shared_ptr<execute_dance_interfaces::srv::IsDancing::Response> response);

private:
    // ========== Movement Execution Methods ==========
    
    /**
     * @brief Send movement to YARP queue for delayed execution
     * @param time Movement duration in seconds
     * @param offset Time offset before execution in milliseconds
     * @param joints Joint angles or movement parameters
     * @param port YARP port for communication with robot part
     * @return true if command sent successfully, false otherwise
     */
    bool SendMovementToQueue(float time, int offset, std::vector<float> joints, yarp::os::Port &port);
    
    /**
     * @brief Send movement for immediate execution
     * @param time Movement duration in seconds
     * @param offset Time offset before execution in milliseconds
     * @param joints Joint angles or movement parameters
     * @param port YARP port for communication with robot part
     * @return true if command sent successfully, false otherwise
     */
    bool SendMovementNow(float time, int offset, std::vector<float> joints, yarp::os::Port &port);
    
    /**
     * @brief Send action command to YARP Actions Player
     * @param actionName Name of the action to execute
     * @return true if command sent successfully, false otherwise
     */
    bool SendMovementToYAP(const std::string &actionName);

    // ========== Pointing Functionality Methods ==========
    
    /**
     * @brief Execute pointing movement towards an artwork
     * @param movement Movement response containing pointing parameters
     * @return true if pointing executed successfully, false otherwise
     * 
     * This method processes pointing_command movements by:
     * 1. Extracting artwork name from joints parameter
     * 2. Looking up artwork coordinates
     * 3. Transforming coordinates to robot reference frame
     * 4. Executing Cartesian pointing movement
     */
    bool ExecutePointingMovement(const std::shared_ptr<dance_interfaces::srv::GetMovement::Response> movement);
    
    /**
     * @brief Extract artwork name from movement joints parameter
     * @param joints Vector where joints[0] contains the artwork index as float
     * @return String name of the artwork to point to
     * 
     * **IMPORTANT**: This function is ONLY used for pointing_command movements.
     * For
     */
    std::string extractArtworkName(const std::vector<float>& joints);
    
    /**
     * @brief Transform coordinates from map frame to robot frame
     * @param mapCoords 3D coordinates in map reference frame [x, y, z]
     * @return 3D coordinates in robot reference frame [x, y, z]
     * 
     * Uses current robot pose from AMCL to transform artwork coordinates
     * from the global map frame to the robot's local coordinate system.
     */
    std::vector<double> transformMapToRobot(const std::vector<double>& mapCoords);
    
    /**
     * @brief Send Cartesian command to robot arm controller
     * @param coords Target coordinates for robot end-effector [x, y, z]
     * @return true if command sent successfully, false otherwise
     */
    bool sendCartesianCommand(const std::vector<double>& coords);

    // ========== Task Execution Methods ==========
    
    /**
     * @brief Execute dance task in separate thread
     * @param request Execute dance request with task parameters
     */
    void executeTask(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request);
    
    /**
     * @brief Timer task for time-based operations
     * @param time Duration for timer task
     */
    void timerTask(float time);

    // ========== Core Component Data ==========
    
    /// Current dance name being executed
    std::string m_danceName;
    
    /// ROS2 node shared pointer
    rclcpp::Node::SharedPtr m_node;
    
    /// Execute dance service server
    rclcpp::Service<execute_dance_interfaces::srv::ExecuteDance>::SharedPtr m_executeDanceService;
    
    /// Is dancing status service server
    rclcpp::Service<execute_dance_interfaces::srv::IsDancing>::SharedPtr m_isDancingService;
    
    /// Map of robot parts to YARP CTP service ports
    std::map<std::string, yarp::os::Port &> m_pCtpService;
    
    /// Thread for dance execution
    std::thread m_threadExecute;
    
    /// Thread for timer operations
    std::thread m_threadTimer;
    
    /// Mutex for timer synchronization
    std::mutex m_timerMutex;
    
    /// Timer task active flag
    bool m_timerTask{false};

    // ========== YARP Actions Players ==========
    
    /// YARP Actions Player client port name
    std::string yAPClientPortName;
    
    /// YARP port for Actions Player communication
    yarp::os::Port m_yAPClientPort;

    // ========== Robot Localization ==========
    
    /// AMCL pose subscription for robot localization
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_amclSub;
    
    /// Current robot X position in map frame
    double m_currentX{0.0};
    
    /// Current robot Y position in map frame
    double m_currentY{0.0};
    
    /// Current robot yaw orientation in map frame
    double m_currentYaw{0.0};

    // ========== Points of Interest and Artworks ==========
    
    /// Map of POI names to 2D coordinates (x, y)
    std::map<std::string, std::pair<double, double>> m_poiCoords;
    
    /// Map of artwork names to 3D coordinates (x, y, z)
    std::map<std::string, std::vector<double>> m_artworkCoords;

    // ========== YARP Cartesian Controller ==========
    
    /// YARP polydriver for Cartesian control
    yarp::dev::PolyDriver m_cartesianClient;
    
    /// YARP Cartesian control interface
    yarp::dev::ICartesianControl* m_cartesianCtrl{nullptr};

    // ========== Callback and Helper Methods ==========
    
    /**
     * @brief Callback for AMCL pose updates
     * @param msg AMCL pose message with covariance
     * 
     * Updates internal robot pose variables (m_currentX, m_currentY, m_currentYaw)
     * used for coordinate transformations in pointing movements.
     */
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    
    /**
     * @brief Load POI coordinates from configuration file
     * @param filename Path to JSON file containing POI coordinates
     * @return Map of POI names to 2D coordinates
     * 
     * Loads Points of Interest coordinates from a JSON configuration file.
     * Expected format: {"pois": {"poi_name": {"x": value, "y": value}}}
     */
    std::map<std::string, std::pair<double, double>> loadPoiCoordinates(const std::string& filename);
    
    /**
     * @brief Load artwork coordinates from configuration file
     * @param filename Path to JSON file containing artwork coordinates
     * @return Map of artwork names to 3D coordinates
     * 
     * Loads artwork coordinates from a JSON configuration file.
     * Expected format: {"artworks": {"artwork_name": {"x": value, "y": value, "z": value}}}
     */
    std::map<std::string, std::vector<double>> loadArtworkCoordinates(const std::string& filename);
};
