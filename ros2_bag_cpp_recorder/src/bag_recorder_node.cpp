#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rclcpp/serialization.hpp>
#include <memory>  // For std::make_shared

// Include the message definitions we want to record
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

class BagRecorderNode : public rclcpp::Node
{
public:
  BagRecorderNode()
  : Node("bag_recorder_node")
  {
    // Create the rosbag2 writer and open a new bag file
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open("my_cpp_bag");

    // List of topics and their ROS message types (as strings)
    std::vector<std::pair<std::string, std::string>> topics = {
      {"/camera/depth/color/points",            "sensor_msgs/msg/PointCloud2"},
      {"/amcl_pose",                             "geometry_msgs/msg/PoseWithCovarianceStamped"},
      {"/cmd_vel",                               "geometry_msgs/msg/Twist"},
      {"/goal_pose",                             "geometry_msgs/msg/PoseStamped"},
      {"/laser_local",                           "sensor_msgs/msg/LaserScan"},
      {"/odometry",                              "nav_msgs/msg/Odometry"},
      {"/navigate_to_pose/_action/status",       "action_msgs/msg/GoalStatusArray"},
      {"/navigate_to_pose/_action/feedback",     "nav2_msgs/action/NavigateToPose_FeedbackMessage"}
    };

    // Get all currently available topics on the ROS graph
    auto available = this->get_topic_names_and_types();

    // Loop over each topic we want to record
    for (auto const & [name, type] : topics) {
      // Skip if the topic is not currently published
      if (!available.count(name)) {
        RCLCPP_WARN(get_logger(),
                    "Topic '%s' not available, skipping.", name.c_str());
        continue;
      }

      // For each known type string, call our template function
      if (type == "sensor_msgs/msg/PointCloud2") {
        subscribe_and_serialize<sensor_msgs::msg::PointCloud2>(name, type);
      } else if (type == "geometry_msgs/msg/PoseWithCovarianceStamped") {
        subscribe_and_serialize<geometry_msgs::msg::PoseWithCovarianceStamped>(name, type);
      } else if (type == "geometry_msgs/msg/Twist") {
        subscribe_and_serialize<geometry_msgs::msg::Twist>(name, type);
      } else if (type == "geometry_msgs/msg/PoseStamped") {
        subscribe_and_serialize<geometry_msgs::msg::PoseStamped>(name, type);
      } else if (type == "sensor_msgs/msg/LaserScan") {
        subscribe_and_serialize<sensor_msgs::msg::LaserScan>(name, type);
      } else if (type == "nav_msgs/msg/Odometry") {
        subscribe_and_serialize<nav_msgs::msg::Odometry>(name, type);
      } else if (type == "action_msgs/msg/GoalStatusArray") {
        subscribe_and_serialize<action_msgs::msg::GoalStatusArray>(name, type);
      } else if (type == "nav2_msgs/action/NavigateToPose_FeedbackMessage") {
        subscribe_and_serialize<nav2_msgs::action::NavigateToPose_FeedbackMessage>(name, type);
      } else {
        // Warn if we encounter an unexpected type string
        RCLCPP_WARN(get_logger(),
                    "Type '%s' not handled, skipping topic '%s'.",
                    type.c_str(), name.c_str());
      }
    }
  }

private:
  // Template helper: subscribe to a topic of type MsgT and serialize every incoming message
  template<typename MsgT>
  void subscribe_and_serialize(
    const std::string & topic_name,
    const std::string & topic_type)
  {
    // Create a standard subscription for MsgT
    auto sub = this->create_subscription<MsgT>(
      topic_name, 10,
      [this, topic_name, topic_type](typename MsgT::SharedPtr msg)
      {
        // Allocate a serialized message buffer
        auto ser = std::make_shared<rclcpp::SerializedMessage>();
        // Create a serializer for the specific MsgT
        rclcpp::Serialization<MsgT> serializer;
        // Perform the serialization
        serializer.serialize_message(msg.get(), ser.get());
        // Write the serialized message into the bag file
        writer_->write(ser, topic_name, topic_type, this->now());
      });
    // Store subscription to keep it alive
    subs_.push_back(sub);

    // Log that we have successfully subscribed
    RCLCPP_INFO(get_logger(),
                "Subscribed and serializing %s [%s]",
                topic_name.c_str(), topic_type.c_str());
  }

  // Keep all subscriptions alive
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  // Writer object for rosbag2
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  // Create and run the BagRecorderNode
  rclcpp::spin(std::make_shared<BagRecorderNode>());
  // Shutdown ROS 2 cleanly
  rclcpp::shutdown();
  return 0;
}
