cmake_minimum_required(VERSION 3.8)
project(bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(Boost COMPONENTS coroutine QUIET)
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

add_library(${PROJECT_NAME} 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ROS2Action.h 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ROS2Action.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ROS2Condition.h 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ROS2Condition.cpp
  )
 
set(dependencies  bt_interfaces rclcpp behaviortree_cpp_v3)

# this line to exports the library
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  find_package(bt_interfaces REQUIRED)
  ament_target_dependencies(${PROJECT_NAME} ${dependencies})
  ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)


  install(
    DIRECTORY include/
    DESTINATION include
  )
  
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
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
