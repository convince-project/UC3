cmake_minimum_required(VERSION 3.5)
project(uc3_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY conf
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/support_code
    DESTINATION share/${PROJECT_NAME}
)

ament_package()