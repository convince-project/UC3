# SPDX-FileCopyrightText: 2024-2024 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

add_executable(scxmlgen)
target_sources(scxmlgen
  PRIVATE
    main.cpp
)

#target_include_directories(scxmlgen ${TinyXML_INCLUDE_DIRS})

target_link_libraries(scxmlgen
  PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_dev
    YARP::YARP_init
    ${TinyXML_LIBRARIES}
)


install(
  TARGETS scxmlgen
  COMPONENT utilities
  DESTINATION ${CMAKE_INSTALL_BINDIR}
)

set_property(TARGET scxmlgen PROPERTY FOLDER "Command Line Tools")
