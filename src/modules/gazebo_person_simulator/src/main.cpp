// main.cpp

#include <rclcpp/rclcpp.hpp>
#include "gazebo_person_simulator.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<GazeboPersonSimulator>();
  exec.add_node(node);
  exec.spin();

  node->cleanupAndDeleteAll(std::chrono::milliseconds(3000));
  rclcpp::shutdown();
  return 0;
}
