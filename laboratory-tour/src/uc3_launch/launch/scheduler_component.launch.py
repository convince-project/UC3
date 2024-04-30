from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from os.path import join

def generate_launch_description():
    tour_path_declare = DeclareLaunchArgument(
        'tour_json_path',
        default_value=join(get_package_share_directory('uc3_launch'),'conf/tours.json'),
        description='The path to the tour json file'
    )
    tour_name_declare = DeclareLaunchArgument(
        'tour_name',
        default_value='TOUR_MADAMA_3',
        description='The tour name to be executed'
    )
    log_level_declare = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='The log level'
    )

    declared_arguments = [tour_path_declare, tour_name_declare, log_level_declare]

    tour_path = LaunchConfiguration('tour_json_path')
    tour_name = LaunchConfiguration('tour_name')
    log_level = LaunchConfiguration('log_level')

    node = Node(
        package='scheduler_component',
        executable='scheduler_component',
        output='screen',
        arguments=[tour_path, tour_name,'--ros-args', '--log-level', log_level]
    )

    return LaunchDescription(declared_arguments + [node])
