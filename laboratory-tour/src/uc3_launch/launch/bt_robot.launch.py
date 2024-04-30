from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PythonExpression, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

from os.path import join
from os import listdir
import sys
import importlib.util

def generate_launch_description():

    default_bt_bath = join(get_package_share_directory('uc3_launch'),'conf/BT/bt_failure_management.xml')
    #default_bt_bath = '/usr/local/src/GIT/convince-project/UC3/laboratory-tour/src/uc3_launch/conf/BT/bt_failure_management.xml'
    spec = importlib.util.spec_from_file_location("skillExeNameExtractor", join(get_package_share_directory('uc3_launch'),'support_code/skillExeNameExtractor.py'))
    skillExeNameExtractor = importlib.util.module_from_spec(spec)
    sys.modules["skillExeNameExtractor"] = skillExeNameExtractor
    spec.loader.exec_module(skillExeNameExtractor)
    bt_path_declare = DeclareLaunchArgument(
        'bt_path',
        default_value= default_bt_bath,
        description='The path to the bt json file'
    )
    declared_arguments = [bt_path_declare]

    bt_path = LaunchConfiguration('bt_path')
    skill_to_exclude = ['check_network_skill','visitors_following_robot_skill',
                        'is_checking_for_people_skill','narrate_poi_skill',]

    path_to_parse = default_bt_bath
    for arg in sys.argv:
        if arg.startswith('bt_path:='):
            path_to_parse = arg.split(':=')[1]
            break
    skills_to_launch, dummy_arguments = skillExeNameExtractor.get_skills_nodes(path_to_parse,skill_to_exclude)

    # Components
    components = [
        Node(
            package='alarm_component',
            executable='alarm_component'
        ),
        Node(
            package='navigation_component',
            executable='navigation_component'
        ),
        Node(
            package='check_network_component',
            executable='check_network_component'
        ),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('uc3_launch'), "/launch/scheduler_component.launch.py"]),
        launch_arguments={
            "tour_json_path": PathJoinSubstitution([FindPackageShare('uc3_launch'), "conf/tours.json"]),
            "tour_name": "TOUR_MADAMA_3",
            "log_level": "debug"
        }.items())]

    # Skills
    nodes = []
    for skill in skills_to_launch:
        nodes += [Node(
            package=skill['package_name'],
            executable=skill['executable_name']
        )]

    # Dummy skills
    for i in range(len(dummy_arguments)):
        nodes += [IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('uc3_launch'), "/launch/dummy_skill.launch.py"]),
        launch_arguments=dummy_arguments[i].items())]

    # Bt executable
    nodes += [IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('uc3_launch'), "/launch/bt_executable.launch.py"]),
        launch_arguments={
            "bt_path": bt_path
        }.items())]

    return LaunchDescription(declared_arguments+components+nodes)

