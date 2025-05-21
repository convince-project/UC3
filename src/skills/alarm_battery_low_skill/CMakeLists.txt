cmake_minimum_required(VERSION 3.16)
project(alarm_battery_low_skill)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bt_interfaces_dummy REQUIRED)

find_package(Qt6 COMPONENTS Core Scxml StateMachine REQUIRED)

add_executable(alarm_battery_low_skill 
  src/main.cpp
  src/AlarmBatteryLowSkill.cpp
)

if (NOT Qt6_FOUND)
  message("qt6 not found")
endif()

ament_target_dependencies(alarm_battery_low_skill
  rclcpp
  sensor_msgs
  bt_interfaces_dummy
  std_msgs
  rclcpp_action
)

target_link_libraries(alarm_battery_low_skill Qt6::Core Qt6::Scxml Qt6::StateMachine
  ${rclcpp_LIBRARIES}
)

target_include_directories(alarm_battery_low_skill
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_sources(alarm_battery_low_skill PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/AlarmBatteryLowSkill.h
)

install(TARGETS
  alarm_battery_low_skill
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

qt6_add_statecharts(alarm_battery_low_skill ${CMAKE_CURRENT_SOURCE_DIR}/src/AlarmBatteryLowSkillSM.scxml)

ament_package()
