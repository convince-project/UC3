# SPDX-FileCopyrightText: 2024-2024 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause
cmake_minimum_required(VERSION 3.16)
project(scxmlgen)

find_package(tinyxml2 CONFIG REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${TinyXML2_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE tinyxml2::tinyxml2)

install(
  TARGETS ${PROJECT_NAME}
  COMPONENT utilities
  DESTINATION ${CMAKE_INSTALL_BINDIR}
)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Command Line Tools")





