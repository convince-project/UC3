from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    package_name_declare = DeclareLaunchArgument(
        'package_name',
        description='The package name'
    )
    executable_name_declare = DeclareLaunchArgument(
        'executable_name',
        default_value=LaunchConfiguration('package_name'),
        description='The executable name'
    )
    skill_name_declare = DeclareLaunchArgument(
        'skill_name',
        description='The skill name to be executed'
    )
    default_status_declare = DeclareLaunchArgument(
        'default_status',
        description='The skill default status'
    )

    declared_arguments = [package_name_declare, executable_name_declare, skill_name_declare, default_status_declare]

    package_name = LaunchConfiguration('package_name')
    executable_name = LaunchConfiguration('executable_name')
    skill_name = LaunchConfiguration('skill_name')
    default_status = LaunchConfiguration('default_status')

    if executable_name is None:
        executable_name = package_name

    # Node
    dummy_action_node = Node(
        package=package_name,
        executable=executable_name,
        arguments=['--skill_name',skill_name,'--default_status',default_status]
    )

    return LaunchDescription(declared_arguments + [dummy_action_node])
