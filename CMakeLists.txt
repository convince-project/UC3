cmake_minimum_required(VERSION 3.12)

project(convince_uc3 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(YCM REQUIRED)
find_package(YARP COMPONENTS os idl_tools dev REQUIRED)

yarp_configure_external_installation(${PROJECT_NAME} WITH_PLUGINS)

add_subdirectory(conf/dialog_app)