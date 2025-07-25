cmake_minimum_required(VERSION 3.16)
project(say_follow_me_skill)
# set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bt_interfaces_dummy REQUIRED)

find_package(text_to_speech_interfaces REQUIRED)
find_package(scheduler_interfaces REQUIRED)
find_package(Qt6 COMPONENTS Core Scxml StateMachine  REQUIRED)

add_executable(${PROJECT_NAME} )

if (NOT Qt6_FOUND)
  message("qt6 not found")
endif()

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_target_dependencies(${PROJECT_NAME} 
  std_msgs
  bt_interfaces_dummy 
  rclcpp 
  rclcpp_action 
  text_to_speech_interfaces 
  scheduler_interfaces 
  
  )
target_link_libraries(${PROJECT_NAME} Qt6::Core Qt6::Scxml Qt6::StateMachine)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_sources( ${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SayFollowMeSkill.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/SayFollowMeSkill.h
  )


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
qt6_add_statecharts(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/SayFollowMeSkillSM.scxml)

ament_package()
