// gazebo_person_simulator.cpp

#include "gazebo_person_simulator.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
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
  // determines tick period
  tick_period_ms_ = this->declare_parameter<int>("tick_period_ms", 500); // 2 Hz

  // default DAE model filename
  default_model_filename_ = this->declare_parameter<std::string>(
    "default_model_filename", "Children/c_casual/sit.dae");

  // default radius of the circular trajectory
  auto_path_radius_ = this->declare_parameter<double>("auto_path_radius", 2.0);

  // Needed services
  make_model_client_ = this->create_client<simulated_world_nws_ros2_msgs::srv::MakeModel>("/world/makeModel");
  delete_obj_client_ = this->create_client<simulated_world_nws_ros2_msgs::srv::DeleteObject>("/world/deleteObject");
  set_pose_client_ = this->create_client<simulated_world_nws_ros2_msgs::srv::SetPose>("/world/setPose");

  loadPersonsFromCSV("/home/user1/congestion-coverage-plan/data/datasets/madama/madama3_september.csv");

  last_tick_ = this->now();
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(tick_period_ms_)),
    std::bind(&GazeboPersonSimulator::tick, this));

}

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

  if (!make_model_client_->service_is_ready() || !set_pose_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Waiting for services /world/makeModel and /world/setPose...");
    last_tick_ = this->now();
    return;
  }

  updateValues();
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

void GazeboPersonSimulator::sendMakeModel(const PersonState& st) {
  auto req = std::make_shared<simulated_world_nws_ros2_msgs::srv::MakeModel::Request>();
  req->id = std::to_string(st.id);
  req->filename = st.model_filename;

  req->pose.x = st.x;
  req->pose.y = st.y;
  req->pose.z = 0.0;
  req->pose.roll = 0.0;
  req->pose.pitch = 0.0;
  req->pose.yaw = wrapAngle(st.yaw + M_PI / 2.0);

  RCLCPP_INFO(this->get_logger(), "Spawning '%s' with model '%s' at (%.2f, %.2f, %.2f)",
              std::to_string(st.id).c_str(), st.model_filename.c_str(), st.x, st.y, 0.0);

  make_model_client_->async_send_request(
    req,
    [this, st](rclcpp::Client<simulated_world_nws_ros2_msgs::srv::MakeModel>::SharedFuture future) {
      const auto resp = future.get();
      if (!resp->success) {
        RCLCPP_ERROR(this->get_logger(), "makeModel failed for '%s'", std::to_string(st.id).c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Spawned '%s'", std::to_string(st.id).c_str());
      }
    }
  );
}

void GazeboPersonSimulator::sendSetPose(const PersonState& st) {
  auto req = std::make_shared<simulated_world_nws_ros2_msgs::srv::SetPose::Request>();
  req->id = std::to_string(st.id);

  req->pose.x = st.x;
  req->pose.y = st.y;
  req->pose.z = 0.0;
  req->pose.roll = 0.0;
  req->pose.pitch = 0.0;
  req->pose.yaw = wrapAngle(st.yaw + M_PI / 2.0);

  RCLCPP_INFO(this->get_logger(), "Setting pose for '%s' to (%.2f, %.2f, %.2f)",
              std::to_string(st.id).c_str(), st.x, st.y, 0.0);
  set_pose_client_->async_send_request(
    req,
    [this, st](rclcpp::Client<simulated_world_nws_ros2_msgs::srv::SetPose>::SharedFuture future) {
      const auto resp = future.get();
      if (!resp->success) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "SetPose failed for '%s'", std::to_string(st.id).c_str());
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

void GazeboPersonSimulator::loadPersonsFromCSV(const std::string& filename){
  std::ifstream file(filename);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", filename.c_str());
    return;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::istringstream ss(line);
    std::string timestamp_str, id_str, x_str, y_str;

    // Parse CSV line
    if (!std::getline(ss, timestamp_str, ',') ||
        !std::getline(ss, id_str, ',') ||
        !std::getline(ss, x_str, ',') ||
        !std::getline(ss, y_str, ',')) {
      continue;
    }
    double timestamp = std::stod(timestamp_str);
    int id = std::stoi(id_str);
    double x = std::stod(x_str);
    double y = std::stod(y_str);
    auto it = persons_.find(timestamp);
    PersonState st;
    st.id = id;
    st.x = x;
    st.y = y;
    st.model_filename = default_model_filename_;
    if (it == persons_.end()) {
      // Create new person
      //RCLCPP_INFO(this->get_logger(), "Loaded new person '%s' at (%.2f, %.2f)", id_str.c_str(), x, y);
      persons_[timestamp] = std::vector<PersonState>{st};
      //person_order_.push_back(timestamp);
      
    }
    else{
      //RCLCPP_INFO(this->get_logger(), "Loaded existing person '%s' at (%.2f, %.2f)", id_str.c_str(), x, y);
      it->second.push_back(st);
    }

  }

  // UNCOMMENT to print the map persons
  // for (const auto& [timestamp, states] : persons_) {
  //   RCLCPP_INFO(this->get_logger(), "Timestamp '%s':", std::to_string(timestamp).c_str());
  //   for (const auto& state : states) {
  //     RCLCPP_INFO(this->get_logger(), "  - State: (%.2f, %.2f, %.2f)", state.x, state.y, state.yaw);
  //   }
  // }

}

// Function to read positions from CSV file
void GazeboPersonSimulator::updateValues() {
  RCLCPP_INFO(this->get_logger(), "Updating person positions from CSV...");
  std::vector<std::string> updated_ids;

  // order keys by value in a vector of doubles
  std::vector<double> values;
  for (const auto& [timestamp, states] : persons_) {
    
    values.push_back(timestamp);
    
  }
  std::sort(values.begin(), values.end());
  RCLCPP_INFO(this->get_logger(), "Sorted timestamps:");
  std::vector<PersonState> states = persons_[values[counter_tick_ % values.size()]];
  for (const auto& state : states) {
    RCLCPP_INFO(this->get_logger(), "  - State: (%.2f, %.2f, %.2f)", state.x, state.y, state.yaw);
  }
  for (const auto& state : states) {
    RCLCPP_INFO(this->get_logger(), "  Inside for loop");
    std::string id_str = std::to_string(state.id);
    updated_ids.push_back(id_str);

    // print spawned names
    RCLCPP_INFO(this->get_logger(), "Currently spawned names:");
    for (const auto& name : spawned_names_) {
      RCLCPP_INFO(this->get_logger(), "  - %s", name.c_str());
    }
    if (spawned_names_.find(id_str) != spawned_names_.end())
    {
      //RCLCPP_INFO(this->get_logger(), "Updating position of '%s' to (%.2f, %.2f)", id.c_str(), state.x, state.y);
      //RCLCPP_INFO(this->get_logger(), "Updating position of '%s' to (%.2f, %.2f)", id_str.c_str(), state.x, state.y);
      sendSetPose(state);
      
    }
    else
    {
      spawned_names_.insert(id_str);
      sendMakeModel(state);
      //sleep(1);
    }
  }

  // Process updated IDs
  for (const auto& id : spawned_names_) {
    // Do something with the updated IDs
    auto it = std::find(updated_ids.begin(), updated_ids.end(), id);
    if (it == updated_ids.end()) {
      // ID was updated
      RCLCPP_INFO(this->get_logger(), "ID '%s' was DELETED.", id.c_str());
      auto req = std::make_shared<simulated_world_nws_ros2_msgs::srv::DeleteObject::Request>();
      req->id = id;
      auto future = delete_obj_client_->async_send_request(req).future.share();
      if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        auto resp = future.get();
        if (!resp->success) {
          RCLCPP_WARN(this->get_logger(), "Cleanup: delete failed for '%s'.", id.c_str());
        }
        else{
          // update also the spawned names set
          spawned_names_.erase(id);
        }
      }
    }
  }
  counter_tick_++;
}
