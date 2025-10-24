cmake_minimum_required(VERSION 3.8)
project(point_cloud_component)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(YARP REQUIRED COMPONENTS os)

include_directories(include)

add_executable(point_cloud_component
  src/main.cpp
  src/CameraSafetyComponent.cpp)

ament_target_dependencies(point_cloud_component
  rclcpp sensor_msgs std_msgs)

target_link_libraries(point_cloud_component
  YARP::YARP_os)

install(TARGETS point_cloud_component
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
