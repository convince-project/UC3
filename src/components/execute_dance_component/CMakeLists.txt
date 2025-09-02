################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

cmake_minimum_required(VERSION 3.8)
project(execute_dance_component)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Silence CMP0074 warning
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Custom interfaces
find_package(dance_interfaces REQUIRED)
find_package(execute_dance_interfaces REQUIRED)

# YARP
find_package(YARP 3.7 REQUIRED COMPONENTS os dev)

# Math / TF / msgs
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ------------------------------------------------------------------------------
# Target
# ------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}
  src/ExecuteDanceComponent.cpp
  src/main.cpp
)

# Include dirs (package includes + Eigen + YARP)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${YARP_INCLUDE_DIRS}
)

# Link ROS deps (via ament) + messages
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  dance_interfaces
  execute_dance_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
)

# Link other libraries (YARP, Eigen)
target_link_libraries(${PROJECT_NAME}
  ${YARP_LIBRARIES}
  Eigen3::Eigen
)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Testing
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
