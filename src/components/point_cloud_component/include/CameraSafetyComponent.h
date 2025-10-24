// =========================
// CameraSafetyComponent.h
// =========================
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

#include <mutex>
#include <optional>
#include <string>

/**
 * @brief Node that evaluates a point cloud stream and exposes an is_safe flag over ROS and YARP.
 */
class CameraSafetyComponent : public rclcpp::Node
{
public:
  /**
   * @brief Construct the node and initialise ROS/YARP interfaces and parameters.
   *
   * @param options node options propagated to the base class.
   */
  explicit CameraSafetyComponent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Process an incoming point cloud and update the safety decision.
   *
   * @param msg point cloud received from the configured ROS topic.
   */
  void handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief Check that the supplied cloud exposes x/y/z fields.
   *
   * @param cloud point cloud metadata to inspect.
   * @return true if all three coordinate fields are present.
   */
  bool hasXYZFields(const sensor_msgs::msg::PointCloud2 & cloud) const;

  /**
   * @brief Count how many valid points drop below the configured z-threshold.
   *
   * @param cloud point cloud data to evaluate.
   * @return number of violations, or std::nullopt if the cloud cannot be read.
   */
  std::optional<int> countThresholdViolations(const sensor_msgs::msg::PointCloud2 & cloud) const;

  /**
   * @brief Publish the safety decision on ROS and YARP, persisting the latest result.
   *
   * @param is_safe true when the latest frame is considered safe.
   * @param violation_count number of points violating the configured threshold.
   */
  void publishSafetyState(bool is_safe, int violation_count);

  // Parameters
  std::string input_cloud_topic_;
  double z_min_threshold_ {0.20};
  int min_violation_count_ {10};

  // ROS2 I/O
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safe_state_pub_;

  // YARP output
  yarp::os::BufferedPort<yarp::os::Bottle> yarp_safe_state_port_;

  // State
  std::mutex state_mtx_;
  bool current_safety_flag_ {true};
  int last_violation_count_ {0};
};
