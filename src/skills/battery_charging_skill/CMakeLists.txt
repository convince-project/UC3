cmake_minimum_required(VERSION 3.8)
project(battery_charging_skill)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bt_interfaces_dummy REQUIRED)
find_package(Qt6 COMPONENTS Core Scxml StateMachine REQUIRED)

add_executable(${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  bt_interfaces_dummy
)

target_link_libraries(${PROJECT_NAME}
  Qt6::Core
  Qt6::Scxml
  Qt6::StateMachine
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_sources(${PROJECT_NAME}
  PRIVATE
    src/main.cpp
    src/BatteryChargingSkill.cpp
    include/BatteryChargingSkill.h
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

qt6_add_statecharts(${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/BatteryChargingSkillSM.scxml
)

ament_package()
