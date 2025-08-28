################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################



cmake_minimum_required(VERSION 3.8)
set (CMAKE_CXX_STANDARD 17)

project(execute_dance_component)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Silence CMP0074 warning
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# Find packages with components specified explicitly
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dance_interfaces REQUIRED)
find_package(execute_dance_interfaces REQUIRED)
find_package(YARP 3.7 REQUIRED COMPONENTS dev os)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# -----------------------
# Build target
# -----------------------
add_executable(${PROJECT_NAME}
  src/ExecuteDanceComponent.cpp
  src/main.cpp
)

# Include dirs
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})

# Link dependencies (use ament_target_dependencies for ROS packages)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  execute_dance_interfaces
  dance_interfaces
  tf2_ros
  tf2_geometry_msgs
)

# Link other libraries (Eigen, YARP)
target_link_libraries(${PROJECT_NAME}
  ${YARP_LIBRARIES}
  Eigen3::Eigen
)

# Sources already set in add_executable; install target
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()