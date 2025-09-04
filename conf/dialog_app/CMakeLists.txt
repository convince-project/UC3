################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

set(appname convince)

file(GLOB apps      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
file(GLOB conf      ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB prompt      ${CMAKE_CURRENT_SOURCE_DIR}/conf/*_prompt.txt)
file(GLOB robotInterface      ${CMAKE_CURRENT_SOURCE_DIR}/conf/robotInterface/*.xml)

yarp_install(FILES ${apps}    DESTINATION ${CONVINCE-UC3_APPLICATIONS_INSTALL_DIR})
yarp_install(FILES ${conf}    DESTINATION ${CONVINCE-UC3_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${prompt}    DESTINATION ${CONVINCE-UC3_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${robotInterface}    DESTINATION ${CONVINCE-UC3_CONTEXTS_INSTALL_DIR}/${appname}/robotInterface)
