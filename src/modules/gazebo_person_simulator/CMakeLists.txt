cmake_minimum_required(VERSION 3.8)
project(gazebo_person_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(simulated_world_nws_ros2_msgs REQUIRED)

# YAML: in ROS2 è comune usare yaml_cpp_vendor. In molte distro esiste anche yaml-cpp.
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp QUIET)

add_executable(${PROJECT_NAME}
  src/gazebo_person_simulator.cpp
  src/main.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  simulated_world_nws_ros2_msgs
  yaml_cpp_vendor
)

# Se disponibile, linka esplicitamente yaml-cpp (risolve i tuoi undefined reference)
if(yaml-cpp_FOUND)
  target_link_libraries(${PROJECT_NAME} yaml-cpp)
endif()

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
