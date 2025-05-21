cmake_minimum_required(VERSION 3.8)
project(bt_executable)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set (dependencies bt_nodes bt_interfaces_dummy rclcpp behaviortree_cpp_v3)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(bt_nodes REQUIRED)
find_package(Boost COMPONENTS coroutine QUIET)
find_package(rclcpp REQUIRED)
find_package(bt_interfaces_dummy REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp )
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
# configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../BT/simple_bt.xml simple_bt.xml COPYONLY)


install(TARGETS ${PROJECT_NAME}
DESTINATION lib/${PROJECT_NAME})

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
