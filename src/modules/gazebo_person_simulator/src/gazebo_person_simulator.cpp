// gazebo_person_simulator.cpp

#include "gazebo_person_simulator.hpp"

#include <cmath>
#include <unordered_map>
#include <utility>

using namespace std::chrono_literals;

// To limit dt
double GazeboPersonSimulator::clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

// To remove inconsistencies in angle representation
double GazeboPersonSimulator::wrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

GazeboPersonSimulator::GazeboPersonSimulator()
: rclcpp::Node("gazebo_person_simulator_ros2")
{
  yaml_path_ = this->declare_parameter<std::string>(
    "yaml_path",
    "/home/user1/UC3/src/modules/gazebo_person_simulator/resource/persons.yaml");

  // box params
  box_w_ = this->declare_parameter<double>("box_width", 0.5);
  box_h_ = this->declare_parameter<double>("box_height", 0.5);
  box_t_ = this->declare_parameter<double>("box_thickness", 0.5);
  box_z_ = this->declare_parameter<double>("box_z", 0.25);
  box_z_ = this->declare_parameter<double>("person_z", 0.0);

  // determines tick period
  tick_period_ms_ = this->declare_parameter<int>("tick_period_ms", 50); // 20 Hz

  // behaviour defaults
  default_speed_ = this->declare_parameter<double>("default_speed", 0.5);

  // distance from the waypoints
  default_arrive_radius_ = this->declare_parameter<double>("default_arrive_radius", 0.25);

  // max rotational velocity
  default_max_yaw_rate_ = this->declare_parameter<double>("default_max_yaw_rate", 2.0);

  // if true continue the path in loop
  default_loop_path_ = this->declare_parameter<bool>("default_loop_path", true);

  // if true stop when the last way_point is reached
  default_stop_at_end_ = this->declare_parameter<bool>("default_stop_at_end", false);

  // default DAE model filename
  default_model_filename_ = this->declare_parameter<std::string>(
    "default_model_filename", "Children/c_casual/sit.dae");

  // default radius of the circular trajectory
  auto_path_radius_ = this->declare_parameter<double>("auto_path_radius", 2.0);

  // Needed services
  make_box_client_ = this->create_client<simulated_world_nws_ros2_msgs::srv::MakeBox>("/world/makeBox");
  make_model_client_ = this->create_client<simulated_world_nws_ros2_msgs::srv::MakeModel>("/world/makeModel");
  delete_obj_client_ = this->create_client<simulated_world_nws_ros2_msgs::srv::DeleteObject>("/world/deleteObject");
  set_pose_client_ = this->create_client<simulated_world_nws_ros2_msgs::srv::SetPose>("/world/setPose");

  loadPersons(yaml_path_);

  last_tick_ = this->now();
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(tick_period_ms_),
    std::bind(&GazeboPersonSimulator::tick, this));

  RCLCPP_INFO(this->get_logger(), "Started. Loaded %zu persons from: %s",
              person_order_.size(), yaml_path_.c_str());
}


// function which loads all the parameters from the yaml file
void GazeboPersonSimulator::loadPersons(const std::string& filename) {
  persons_.clear();
  person_order_.clear();

  YAML::Node config = YAML::LoadFile(filename);
  if (!config["persons"]) {
    throw std::runtime_error("YAML missing 'persons' key");
  }

  // global YAML param
  num_points_per_person_ = config["num_points_per_person"]
    ? config["num_points_per_person"].as<int>()
    : num_points_per_person_;

  for (auto it = config["persons"].begin(); it != config["persons"].end(); ++it) {
    const std::string name = it->first.as<std::string>();
    const YAML::Node p = it->second;

    PersonState st;
    st.id = p["id"] ? p["id"].as<int>() : 0;

    st.x = p["start_x"] ? p["start_x"].as<double>() : 0.0;
    st.y = p["start_y"] ? p["start_y"].as<double>() : 0.0;
    st.start_x = st.x;
    st.start_y = st.y;

    st.yaw = p["start_yaw"] ? p["start_yaw"].as<double>() : 0.0;

    st.speed = p["speed"] ? p["speed"].as<double>() : default_speed_;
    st.arrive_radius = p["arrive_radius"] ? p["arrive_radius"].as<double>() : default_arrive_radius_;
    st.max_yaw_rate = p["max_yaw_rate"] ? p["max_yaw_rate"].as<double>() : default_max_yaw_rate_;
    st.loop_path = p["loop_path"] ? p["loop_path"].as<bool>() : default_loop_path_;
    st.stop_at_end = p["stop_at_end"] ? p["stop_at_end"].as<bool>() : default_stop_at_end_;
    st.model_filename = p["model_filename"] ? p["model_filename"].as<std::string>() : default_model_filename_;

    /*
    It is possible to add in the yaml file a path using the following structure:
     ----------- path: [[x,y],[x,y],...] or path: - {x:..., y:...} ----------
     This coordinates describe the way_point, indeed they define the trajectory the person should do.
     If the path is not specified, the person will move in a circular path around the start position created
     in the function generateCircularPath.
    */
    if (p["path"] && p["path"].IsSequence()) {
      for (const auto& wpNode : p["path"]) {
        Waypoint wp;
        if (wpNode.IsSequence() && wpNode.size() >= 2) {
          wp.x = wpNode[0].as<double>();
          wp.y = wpNode[1].as<double>();
        } else if (wpNode.IsMap() && wpNode["x"] && wpNode["y"]) {
          wp.x = wpNode["x"].as<double>();
          wp.y = wpNode["y"].as<double>();
        } else {
          RCLCPP_WARN(this->get_logger(), "Invalid waypoint for '%s' - skipping", name.c_str());
          continue;
        }
        st.path.push_back(wp);
      }
    }

    // auto-path if missing
    if (st.path.empty()) {
      st.path = generateCircularPath(st.start_x, st.start_y, auto_path_radius_, num_points_per_person_);
      st.loop_path = true;
      st.stop_at_end = false;

      RCLCPP_INFO(this->get_logger(),
        "Auto-path generated for '%s' with %zu points (radius=%.2f)",
        name.c_str(), st.path.size(), auto_path_radius_);
    }

    persons_[name] = st;
    person_order_.push_back(name);

    RCLCPP_INFO(this->get_logger(),
      "Loaded %s (x=%.2f y=%.2f speed=%.2f path_pts=%zu model=%s)",
      name.c_str(), st.x, st.y, st.speed, st.path.size(), st.model_filename.c_str());
  }

  RCLCPP_INFO(this->get_logger(),
    "num_points_per_person=%d (from YAML or default)", num_points_per_person_);
}


/*
function which generates a circular path if the way_points are not explicitely specified in the yaml file
x = cx + radius*cos(a)
y = cy + radius*sin(a)
where a = (2.0 * M_PI) * (i / num_wp);

Giro completo --> 2pi, diviso in n fette.
Ogni punto quindi è distanziato da Delta(a) = 2pi/n
Quindi il punto iesimo è a = i * Delta(a)
*/
std::vector<Waypoint> GazeboPersonSimulator::generateCircularPath(double cx, double cy, double radius, int n) {
  std::vector<Waypoint> out;
  if (n < 3) n = 3;
  out.reserve(static_cast<size_t>(n));

  for (int i = 0; i < n; ++i) {
    const double a = (2.0 * M_PI) * (static_cast<double>(i) / static_cast<double>(n));
    out.push_back(Waypoint{cx + radius * std::cos(a), cy + radius * std::sin(a)});
  }
  return out;
}

// This function is called periodically by the timer
void GazeboPersonSimulator::tick() {
  if (!make_box_client_->service_is_ready() || !set_pose_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Waiting for services /world/makeBox and /world/setPose...");
    last_tick_ = this->now();
    return;
  }

  if (!make_model_client_->service_is_ready() || !set_pose_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Waiting for services /world/makeModel and /world/setPose...");
    last_tick_ = this->now();
    return;
  }

  if (person_order_.empty()) return;

  const auto now_t = this->now();
  double dt = (now_t - last_tick_).seconds();
  last_tick_ = now_t;
  if (dt <= 0.0) return;
  dt = clamp(dt, 0.0, 0.2);

  // spawn all the person
  for (const auto& name : person_order_) {
    auto& st = persons_[name];
    if (!st.spawned && !st.spawn_in_flight) {
      st.spawn_in_flight = true;
      sendMakeBox(name, st);
      // sendMakeModel(name, st);
    }
  }

  // move & publish
  for (const auto& name : person_order_) {
    auto& st = persons_[name];
    if (!st.spawned) continue;

    stepPerson(st, dt);
    sendSetPose(name, st);
  }
}


void GazeboPersonSimulator::stepPerson(PersonState& st, double dt) {
  if (st.path.empty()) return;

  if (st.wp_idx >= st.path.size()) st.wp_idx = 0;

  // extract the waypoint
  const auto& wp = st.path[st.wp_idx];

  double dx = wp.x - st.x;
  double dy = wp.y - st.y;
  double dist = std::hypot(dx, dy);

  // If the person is sufficiently near to the waypoint go to the next wp
  if (dist < st.arrive_radius) {
    // check if arrived at the end of the path
    if (st.wp_idx + 1 < st.path.size()) {
      st.wp_idx++;
    } else {
      if (st.stop_at_end) return;
      if (st.loop_path) st.wp_idx = 0;
      else st.wp_idx = st.path.size() - 1;
    }
  }

  // update parameters
  const auto& wp2 = st.path[st.wp_idx];
  dx = wp2.x - st.x;
  dy = wp2.y - st.y;
  dist = std::hypot(dx, dy);
  if (dist < 1e-6) return;

  const double desired_yaw = std::atan2(dy, dx);
  const double yaw_err = wrapAngle(desired_yaw - st.yaw);
  const double max_dyaw = st.max_yaw_rate * dt;
  st.yaw = wrapAngle(st.yaw + clamp(yaw_err, -max_dyaw, max_dyaw));

  // Move in the direction of the desired waypoint at current orientation
  const double move_dx = dx / dist * st.speed * dt;
  const double move_dy = dy / dist * st.speed * dt;
  st.x += move_dx;
  st.y += move_dy;
}

void GazeboPersonSimulator::sendMakeBox(const std::string& name, PersonState& st) {
  auto req = std::make_shared<simulated_world_nws_ros2_msgs::srv::MakeBox::Request>();
  req->id = name;

  req->width = box_w_;
  req->height = box_h_;
  req->thickness = box_t_;

  req->pose.x = st.x;
  req->pose.y = st.y;
  req->pose.z = box_z_;
  req->pose.roll = 0.0;
  req->pose.pitch = 0.0;
  req->pose.yaw = st.yaw;

  req->color.r = 255;
  req->color.g = 255;
  req->color.b = 255;

  req->frame_name = "";
  req->gravity_enable = false;
  req->collision_enable = false;

  RCLCPP_INFO(this->get_logger(), "Spawning '%s' at (%.2f, %.2f, %.2f)",
              name.c_str(), st.x, st.y, box_z_);

  make_box_client_->async_send_request(
    req,
    [this, name](rclcpp::Client<simulated_world_nws_ros2_msgs::srv::MakeBox>::SharedFuture future) {
      const auto resp = future.get();
      auto it = persons_.find(name);
      if (it == persons_.end()) return;

      if (!resp->success) {
        RCLCPP_ERROR(this->get_logger(), "makeBox failed for '%s'", name.c_str());
        it->second.spawned = false;
      } else {
        it->second.spawned = true;
        spawned_names_.insert(name);
        RCLCPP_INFO(this->get_logger(), "Spawned '%s'", name.c_str());
      }
      it->second.spawn_in_flight = false;
    }
  );
}

void GazeboPersonSimulator::sendMakeModel(const std::string& name, PersonState& st) {
  auto req = std::make_shared<simulated_world_nws_ros2_msgs::srv::MakeModel::Request>();
  req->id = name;
  req->filename = st.model_filename;

  req->pose.x = st.x;
  req->pose.y = st.y;
  req->pose.z = box_z_;
  req->pose.roll = 0.0;
  req->pose.pitch = 0.0;
  req->pose.yaw = wrapAngle(st.yaw + M_PI / 2.0);

  RCLCPP_INFO(this->get_logger(), "Spawning '%s' with model '%s' at (%.2f, %.2f, %.2f)",
              name.c_str(), st.model_filename.c_str(), st.x, st.y, box_z_);

  make_model_client_->async_send_request(
    req,
    [this, name](rclcpp::Client<simulated_world_nws_ros2_msgs::srv::MakeModel>::SharedFuture future) {
      const auto resp = future.get();
      auto it = persons_.find(name);
      if (it == persons_.end()) return;

      if (!resp->success) {
        RCLCPP_ERROR(this->get_logger(), "makeModel failed for '%s'", name.c_str());
        it->second.spawned = false;
      } else {
        it->second.spawned = true;
        spawned_names_.insert(name);
        RCLCPP_INFO(this->get_logger(), "Spawned '%s'", name.c_str());
      }
      it->second.spawn_in_flight = false;
    }
  );
}

void GazeboPersonSimulator::sendSetPose(const std::string& name, const PersonState& st) {
  auto req = std::make_shared<simulated_world_nws_ros2_msgs::srv::SetPose::Request>();
  req->id = name;

  req->pose.x = st.x;
  req->pose.y = st.y;
  req->pose.z = box_z_;
  req->pose.roll = 0.0;
  req->pose.pitch = 0.0;
  req->pose.yaw = wrapAngle(st.yaw + M_PI / 2.0);

  set_pose_client_->async_send_request(
    req,
    [this, name](rclcpp::Client<simulated_world_nws_ros2_msgs::srv::SetPose>::SharedFuture future) {
      const auto resp = future.get();
      if (!resp->success) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "SetPose failed for '%s'", name.c_str());
      }
    }
  );
}

// This function deletes all the created person before closing the module
void GazeboPersonSimulator::cleanupAndDeleteAll(std::chrono::milliseconds timeout) {
  // If nothing has been spawned do nothing
  if (spawned_names_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Cleanup: no spawned persons to delete.");
    return;
  }

  // if (!delete_obj_client_->service_is_ready()) {
  //   RCLCPP_WARN(this->get_logger(), "Cleanup: /world/deleteObject not ready, skipping delete.");
  //   return;
  // }

  RCLCPP_INFO(this->get_logger(), "Cleanup: deleting %zu persons...", spawned_names_.size());

  // Send all requests
  std::vector<rclcpp::Client<simulated_world_nws_ros2_msgs::srv::DeleteObject>::SharedFuture> futures;
  futures.reserve(spawned_names_.size());

  for (const auto& name : spawned_names_) {
    auto req = std::make_shared<simulated_world_nws_ros2_msgs::srv::DeleteObject::Request>();
    req->id = name;
    futures.push_back(delete_obj_client_->async_send_request(req).future.share());
  }

  // Wait for completion, but need to process callbacks -> spin_some
  rclcpp::executors::SingleThreadedExecutor tmp_exec;
  tmp_exec.add_node(this->shared_from_this());

  const auto start = this->now();
  while (rclcpp::ok()) {
    bool all_done = true;
    for (auto & f : futures) {
      if (f.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        all_done = false;
        break;
      }
    }
    if (all_done) break;

    // process service callbacks
    tmp_exec.spin_some();

    // timeout
    if ((this->now() - start).seconds() * 1000.0 > static_cast<double>(timeout.count())) {
      RCLCPP_WARN(this->get_logger(), "Cleanup: timeout reached, some deletes may be pending.");
      break;
    }
  }

  // Log results for those that are ready
  for (size_t i = 0; i < futures.size(); ++i) {
    if (futures[i].wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      auto resp = futures[i].get();
      if (!resp->success) {
        RCLCPP_WARN(this->get_logger(), "Cleanup: delete failed for one person.");
      }
    }
  }

  spawned_names_.clear();
  RCLCPP_INFO(this->get_logger(), "Cleanup: delete requests sent/completed.");
}
