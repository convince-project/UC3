cmake_minimum_required(VERSION 3.8)
project(bt_executable_reload)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set (dependencies bt_nodes bt_interfaces rclcpp behaviortree_cpp_v3)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(bt_nodes REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bt_interfaces REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
#find_package(ZeroMQ)

add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp )
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
ament_target_dependencies(${PROJECT_NAME} ${dependencies} bt_interfaces)
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

find_package(PkgConfig)
## use pkg-config to get hints for 0mq locations
pkg_check_modules(PC_ZeroMQ QUIET zmq)
## use the hint from above to find where 'zmq.hpp' is located
find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )

## use the hint from above to find the location of libzmq
find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )

# Check if ZeroMQ was found
if (ZeroMQ_INCLUDE_DIR AND ZeroMQ_LIBRARY)
    # Add the include directory to our compile directives
    target_include_directories(${PROJECT_NAME} PUBLIC ${ZeroMQ_INCLUDE_DIR})

    # Link the ZeroMQ library to our project
    target_link_libraries(${PROJECT_NAME} ${ZeroMQ_LIBRARY})

    # Define the ZMQ_FOUND macro
    add_definitions(-DZMQ_FOUND)
else()
    message(WARNING "ZeroMQ not found. The ZMQ-related functionality will be disabled.")
endif()

ament_package()
