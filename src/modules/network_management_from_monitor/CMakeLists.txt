cmake_minimum_required(VERSION 3.5)
project(network_management_from_monitor)

# Attiva warnings moderni se vuoi
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(manage_service_interfaces REQUIRED)
find_package(dialog_interfaces REQUIRED)

add_executable(${PROJECT_NAME} )

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  manage_service_interfaces
  dialog_interfaces
)

target_link_libraries(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_sources( ${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/network_management_from_monitor.cpp
  )

install(TARGETS ${PROJECT_NAME}
DESTINATION lib/${PROJECT_NAME})

ament_package()