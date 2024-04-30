from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from os.path import join

def generate_launch_description():
    default_bt_bath = join(get_package_share_directory('uc3_launch'),'conf/BT/bt_failure_management.xml')
    bt_path_declare = DeclareLaunchArgument(
        'bt_path',
        default_value= default_bt_bath,
        description='The path to the bt json file'
    )
    declared_arguments = [bt_path_declare]

    bt_path = LaunchConfiguration('bt_path')

    node = Node(
        package='bt_executable',
        executable='bt_executable',
        output='screen',
        arguments=[bt_path]
    )

    return LaunchDescription(declared_arguments + [node])
