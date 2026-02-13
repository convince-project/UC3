// gazebo_person_simulator.hpp

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <simulated_world_nws_ros2_msgs/srv/make_box.hpp>
#include <simulated_world_nws_ros2_msgs/srv/delete_object.hpp>
#include <simulated_world_nws_ros2_msgs/srv/set_pose.hpp>

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

struct Waypoint {
  double x{0.0};
  double y{0.0};
};

struct PersonState {
  int id{0};

  // pose
  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  // motion
  double speed{0.5};           // m/s
  double max_yaw_rate{2.0};    // rad/s

  // path following
  std::vector<Waypoint> path;
  size_t wp_idx{0};
  double arrive_radius{0.25};  // m
  bool loop_path{true};
  bool stop_at_end{false};

  // spawn
  bool spawned{false};
  bool spawn_in_flight{false};

  // start (useful for auto-path)
  double start_x{0.0};
  double start_y{0.0};
};

class GazeboPersonSimulator : public rclcpp::Node {
public:
  GazeboPersonSimulator();

  void cleanupAndDeleteAll(std::chrono::milliseconds timeout = std::chrono::milliseconds(3000));
private:
  void loadPersons(const std::string& filename);

  std::vector<Waypoint> generateCircularPath(double cx, double cy, double radius, int n);

  void tick();
  void stepPerson(PersonState& st, double dt);

  void sendMakeBox(const std::string& name, PersonState& st);
  void sendSetPose(const std::string& name, const PersonState& st);

  // helpers
  static double clamp(double v, double lo, double hi);
  static double wrapAngle(double a);

private:
  // Params
  std::string yaml_path_;

  double box_w_{1.0}, box_h_{1.0}, box_t_{1.0}, box_z_{0.5};
  int tick_period_ms_{50};

  double default_speed_{0.5};
  double default_arrive_radius_{0.25};
  double default_max_yaw_rate_{2.0};
  bool default_loop_path_{true};
  bool default_stop_at_end_{false};

  double auto_path_radius_{2.0};
  int num_points_per_person_{7};

  // Services
  rclcpp::Client<simulated_world_nws_ros2_msgs::srv::MakeBox>::SharedPtr make_box_client_;
  rclcpp::Client<simulated_world_nws_ros2_msgs::srv::DeleteObject>::SharedPtr delete_obj_client_;
  rclcpp::Client<simulated_world_nws_ros2_msgs::srv::SetPose>::SharedPtr set_pose_client_;

  // State
  std::unordered_map<std::string, PersonState> persons_;
  std::vector<std::string> person_order_;

  std::unordered_set<std::string> spawned_names_;

  rclcpp::Time last_tick_;
  rclcpp::TimerBase::SharedPtr timer_;
};
