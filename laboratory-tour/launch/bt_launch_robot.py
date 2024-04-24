from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='alarm_battery_low_skill',
            executable='alarm_battery_low_skill'
        ),
        Node(
            package='battery_level_skill',
            executable='battery_level_skill'
        ),
        Node(
            package='alarm_component',
            executable='alarm_component'
        ),
        #Node(
        #    package='bt_executable',
        #    executable='bt_executable',
        #    arguments=[ './src/behavior_tree/BT/bt_failure_management.xml']
        #), 
        Node(
            package='battery_charging_skill',
            executable='battery_charging_skill'
        ),         
        Node(
            package='is_at_charging_station_skill',
            executable='is_at_charging_station_skill'
        ),         
        Node(
            package='go_to_charging_station_skill',
            executable='go_to_charging_station_skill'
        ),
        Node(
            package='navigation_component',
            executable='navigation_component'
        ),
        Node(
            package='check_network_component',
            executable='check_network_component'
        ),
        Node(
            package='scheduler_component',
            executable='scheduler_component', 
            output='screen',
            arguments=['./conf/tours.json', 'TOUR_MADAMA_3','--ros-args', '--log-level', 'debug']
        ),         
        Node(
            package='stop_and_turn_back_skill',
            executable='stop_and_turn_back_skill' 
        ),         
        Node(
            package='is_at_current_poi_skill',
            executable='is_at_current_poi_skill' 
        ),         
        Node(
            package='update_poi_skill',
            executable='update_poi_skill' 
        ),         
        Node(
            package='go_to_current_poi_skill',
            executable='go_to_current_poi_skill', 
            output="screen"
        ),         
        #Node(
        #    package='check_network_skill',
        #    executable='check_network_skill',
        #    output="screen"
        #),         
        Node(
            package='dummy_condition',
            executable='dummy_condition', 
            arguments=['--skill_name', 'VisitorsFollowingRobot', '--default_status', 'SUCCESS']
        ),          
        Node(
            package='dummy_condition',
            executable='dummy_condition', 
            arguments=['--skill_name', 'CheckNetwork', '--default_status', 'SUCCESS']
        ),          
        Node(
            package='dummy_condition',
            executable='dummy_condition', 
            arguments=['--skill_name', 'IsCheckingForPeople', '--default_status', 'SUCCESS']
        ),
        Node(
            package='dummy_action',
            executable='dummy_action', 
            arguments=['--skill_name', 'NarratePOI', '--default_status', 'SUCCESS']
        ), 
        Node(
            package='dummy_action',
            executable='dummy_action', 
            arguments=['--skill_name', 'ManageModulesIfNetworkDown', '--default_status', 'SUCCESS']
        ),
        Node(
            package='dummy_action',
            executable='dummy_action', 
            arguments=['--skill_name', 'SayWhileNavigating', '--default_status', 'SUCCESS']
        ), 
        Node(
            package='dummy_action',
            executable='dummy_action', 
            arguments=['--skill_name', 'SayFollowMe', '--default_status', 'SUCCESS']
        ), 
    ])

