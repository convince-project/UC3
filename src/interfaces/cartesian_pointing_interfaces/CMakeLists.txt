cmake_minimum_required(VERSION 3.16)
project(cartesian_pointing_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PointAt.srv"
  "srv/IsPointing.srv"
)

ament_package()