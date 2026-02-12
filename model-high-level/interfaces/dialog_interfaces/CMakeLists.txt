cmake_minimum_required(VERSION 3.8)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(dialog_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(dialog_interfaces
                            "srv/ManageContext.srv"
                            "srv/SetLanguage.srv"
                            "srv/RememberInteractions.srv"
                            "srv/InterpretCommand.srv"
                            "srv/Answer.srv"
                            "srv/ShortenReply.srv"
                            "srv/IsSpeaking.srv"
                            "srv/SetMicrophone.srv"
                            "srv/SetWebStatus.srv"
                            "srv/OfflineSpeak.srv"
                            "srv/ResetState.srv"
                            "action/Speak.action"
                            "action/WaitForInteraction.action"
                            DEPENDENCIES sensor_msgs
                            LIBRARY_NAME dialog_interfaces
)
ament_export_dependencies(sensor_msgs)


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
