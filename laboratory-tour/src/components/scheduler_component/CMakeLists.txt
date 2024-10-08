################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################



cmake_minimum_required(VERSION 3.8)
set (CMAKE_CXX_STANDARD 17)

project(scheduler_component)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scheduler_interfaces_dummy REQUIRED)
# find_package(log_library REQUIRED)
# find_package(nlohmann_json 3.10.5 REQUIRED)
add_executable(${PROJECT_NAME} )

include(FetchContent)


# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# find_library(LOG_LIBRARY NAMES log_library PATHS ${CMAKE_INSTALL_PREFIX}/lib)
# find_path(LOG_LIBRARY_INCLUDE_DIRS LogLibrary.h PATHS ${CMAKE_INSTALL_PREFIX}/include)
# include_directories(
#   include
#   ${log_library_INCLUDE_DIRS}  # Ensure this line is added
# )

ament_target_dependencies(${PROJECT_NAME} scheduler_interfaces_dummy rclcpp 
# log_library
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_sources( ${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SchedulerComponent.cpp  
  ${CMAKE_CURRENT_SOURCE_DIR}/include/SchedulerComponent.h 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Action.cpp  
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Action.h 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Poi.cpp  
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Poi.h 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Tour.cpp  
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Tour.h 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/TourStorage.cpp  
  ${CMAKE_CURRENT_SOURCE_DIR}/include/TourStorage.h 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)


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
