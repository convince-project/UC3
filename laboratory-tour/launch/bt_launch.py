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
        Node(
            package='bt_executable',
            executable='bt_executable',
            arguments=[ './src/behavior_tree/BT/laboratory_tourv1.xml']
        ), 
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
            package='schduler_component',
            executable='schduler_component', 
            arguments=['./conf/tours.json']
        ),         
        Node(
            package='dummy_condition',
            executable='dummy_condition', 
            arguments=['--skill_name', 'AtCurrentPOI', '--default_status', 'SUCCESS']
        ),         
        Node(
            package='dummy_condition',
            executable='dummy_condition', 
            arguments=['--skill_name', 'VisitorsFollowingRobot', '--default_status', 'FAILURE']
        ),          
        Node(
            package='dummy_condition',
            executable='dummy_condition', 
            arguments=['--skill_name', 'IsCheckingForPeople', '--default_status', 'FAILURE']
        ),
        Node(
            package='dummy_action',
            executable='dummy_action', 
            arguments=['--skill_name', 'NarratePOI', '--default_status', 'SUCCESS']
        ), 
        Node(
            package='dummy_action',
            executable='dummy_action', 
            arguments=['--skill_name', 'UpdatePOI', '--default_status', 'SUCCESS']        ), 
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
            arguments=['--skill_name', 'GoToPOI', '--default_status', 'SUCCESS']
        ), 
        Node(
            package='dummy_action',
            executable='dummy_action', 
            arguments=['--skill_name', 'SayFollowMe', '--default_status', 'SUCCESS']
        ), 
        Node(
            package='dummy_action',
            executable='dummy_action', 
            arguments=['--skill_name', 'StopAndTurnBack', '--default_status', 'SUCCESS']
        )
    ])

