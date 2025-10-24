#include "PointCloudComponent.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudComponent>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
