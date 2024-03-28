################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

cmake_minimum_required(VERSION 3.8)
set (CMAKE_CXX_STANDARD 17)

project(dialog_component)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dialog_interfaces REQUIRED)
#find_package(YCM REQUIRED)
find_package(YARP REQUIRED COMPONENTS dev os sig REQUIRED)

#add_executable(${PROJECT_NAME})

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#ament_target_dependencies(${PROJECT_NAME} dialog_interfaces rclcpp )
#target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})
#
#target_include_directories(${PROJECT_NAME}
#  PUBLIC
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include>)
#target_sources( ${PROJECT_NAME} PRIVATE
#  ${CMAKE_CURRENT_SOURCE_DIR}/src/DialogComponent.cpp  
#  ${CMAKE_CURRENT_SOURCE_DIR}/include/DialogComponent.hpp
#  ${CMAKE_CURRENT_SOURCE_DIR}/include/SpeechTranscriberCallback.hpp
#  ${CMAKE_CURRENT_SOURCE_DIR}/src/Action.cpp
#  ${CMAKE_CURRENT_SOURCE_DIR}/src/Poi.cpp
#  ${CMAKE_CURRENT_SOURCE_DIR}/src/Tour.cpp
#  ${CMAKE_CURRENT_SOURCE_DIR}/src/TourStorage.cpp
#  ${CMAKE_CURRENT_SOURCE_DIR}/include/Action.h
#  ${CMAKE_CURRENT_SOURCE_DIR}/include/Poi.h
#  ${CMAKE_CURRENT_SOURCE_DIR}/include/Tour.h
#  ${CMAKE_CURRENT_SOURCE_DIR}/include/TourStorage.h
#  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

include_directories(include)

add_executable(${PROJECT_NAME}  src/DialogComponent.cpp  
                                src/Action.cpp
                                src/Poi.cpp
                                src/Tour.cpp
                                src/TourStorage.cpp
                                src/main.cpp)
ament_target_dependencies(${PROJECT_NAME} 
"YARP"
"rclcpp"
"dialog_interfaces"
)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
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

ament_export_include_directories(include)

ament_package()
