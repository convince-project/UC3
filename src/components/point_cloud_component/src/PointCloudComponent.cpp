#include "PointCloudComponent.h"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>

namespace
{
// Default endpoints so the node can run with zero configuration.
constexpr char kDefaultInputTopic[] = "/camera/depth/color/points";
constexpr char kRosSafetyTopic[] = "/CameraSafety/is_safe";
constexpr char kYarpSafetyPort[] = "/CameraSafety/is_safe:o";
}

PointCloudComponent::PointCloudComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("PointCloudComponent", options)
{
  // Declare parameters
  // These parameters let operators tune topic names and safety thresholds at runtime.
  input_cloud_topic_   = this->declare_parameter<std::string>("input_cloud_topic", kDefaultInputTopic);
  z_min_threshold_     = this->declare_parameter<double>("z_min_threshold", 2.20);
  min_violation_count_ = this->declare_parameter<int>("min_violation_count", 10);

  // ROS2 publisher
  // Publish the current safety flag so other ROS nodes can react immediately.
  safe_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(kRosSafetyTopic, 10);

  // YARP init and port open
  if (!yarp::os::Network::checkNetwork())
  {
    RCLCPP_WARN(this->get_logger(), "YARP network not available; YARP output will be disabled");
  }
  else
  {
    // Open only if not already opened (getName() empty before open)
    // This allows running multiple instances with remapped port names.
    if (yarp_safe_state_port_.getName().empty())
    {
      yarp_safe_state_port_.open(kYarpSafetyPort);
    }
  }

  // ROS2 subscriber
  // Subscribe to the point cloud stream; processing happens in handlePointCloud.
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      this->handlePointCloud(msg);
    });

  RCLCPP_INFO(
    this->get_logger(),
    "PointCloudComponent listening to '%s' (z_min_threshold=%.3f, min_violation_count=%d)",
    input_cloud_topic_.c_str(),
    z_min_threshold_,
    min_violation_count_);
}

void PointCloudComponent::handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // The PointCloud2Iterator APIs rely on x/y/z fields; bail out early if the sensor omits them.
  if (!hasXYZFields(*msg))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "PointCloud2 missing x/y/z fields; skipping frame");
    return;
  }

  // Count how many points are inside the forbidden volume.
  const auto violation_count = countThresholdViolations(*msg);
  if (!violation_count.has_value())
  {
    return;
  }

  // A frame is safe when too few points break the threshold.
  const bool is_safe_frame = (*violation_count < min_violation_count_);
  publishSafetyState(is_safe_frame, *violation_count);
}

bool PointCloudComponent::hasXYZFields(const sensor_msgs::msg::PointCloud2 & cloud) const
{
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;

  // Scan the field metadata instead of reading data buffers we may not understand.
  for (const auto & field : cloud.fields)
  {
    if (field.name == "x")
    {
      has_x = true;
    }

    if (field.name == "y")
    {
      has_y = true;
    }

    if (field.name == "z")
    {
      has_z = true;
    }
  }

  return has_x && has_y && has_z;
}

std::optional<int> PointCloudComponent::countThresholdViolations(const sensor_msgs::msg::PointCloud2 & cloud) const
{
  try
  {
    sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");
    const size_t total_points = cloud.width * cloud.height;
    int violations = 0;

    // Only the z coordinate matters for this simple heuristic; skip NaNs and infs.
    for (size_t idx = 0; idx < total_points; ++idx, ++it_z)
    {
      const float z = *it_z;
      if (std::isfinite(z) && z < z_min_threshold_)
      {
        ++violations;
      }
    }

    return violations;
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(this->get_logger(), "Exception reading PointCloud2: %s", e.what());
    return std::nullopt;
  }
}

void PointCloudComponent::publishSafetyState(bool is_safe, int violation_count)
{
  {
    // Remember the latest decision for monitoring or future service calls.
    std::lock_guard<std::mutex> guard(state_mtx_);
    current_safety_flag_ = is_safe;
    last_violation_count_ = violation_count;
  }

  // Broadcast the decision on ROS so automation pipelines can react.
  std_msgs::msg::Bool safe_msg;
  safe_msg.data = is_safe;
  safe_state_pub_->publish(safe_msg);

  if (!yarp_safe_state_port_.getName().empty())
  {
    // Mirror the same information on YARP for legacy integration layers.
    yarp::os::Bottle & payload = yarp_safe_state_port_.prepare();
    payload.clear();
    payload.addString("is_safe");
    payload.addInt32(is_safe ? 1 : 0);
    payload.addString("violations");
    payload.addInt32(violation_count);
    yarp_safe_state_port_.write();
  }
}
