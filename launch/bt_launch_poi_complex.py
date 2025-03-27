from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bt_executable_reload',
            executable='bt_executable_reload',
            arguments=[ './src/behavior_tree/BT/bt_poi_complex.xml']
        ), 
    ])

