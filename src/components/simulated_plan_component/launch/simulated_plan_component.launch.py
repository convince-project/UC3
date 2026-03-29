from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument('time_per_navigation_poi', default_value='30.0'),
        DeclareLaunchArgument('time_per_explain_poi', default_value='40.0'),
        DeclareLaunchArgument('time_per_question_poi', default_value='20.0'),
        DeclareLaunchArgument('battery_drainage_per_poi', default_value='5.0'),
        DeclareLaunchArgument('battery_safety_margin', default_value='10.0'),
        DeclareLaunchArgument('low_battery_threshold', default_value='35.0'),
        DeclareLaunchArgument('critical_battery_threshold', default_value='20.0'),
        DeclareLaunchArgument('moderate_time_pressure_ratio', default_value='1.1'),
        DeclareLaunchArgument('high_time_pressure_ratio', default_value='1.3'),
    ]

    node = Node(
        package='simulated_plan_component',
        executable='simulated_plan_component',
        name='simulated_plan_component',
        output='screen',
        parameters=[
            {
                'time_per_navigation_poi': LaunchConfiguration('time_per_navigation_poi'),
                'time_per_explain_poi': LaunchConfiguration('time_per_explain_poi'),
                'time_per_question_poi': LaunchConfiguration('time_per_question_poi'),
                'battery_drainage_per_poi': LaunchConfiguration('battery_drainage_per_poi'),
                'battery_safety_margin': LaunchConfiguration('battery_safety_margin'),
                'low_battery_threshold': LaunchConfiguration('low_battery_threshold'),
                'critical_battery_threshold': LaunchConfiguration('critical_battery_threshold'),
                'moderate_time_pressure_ratio': LaunchConfiguration('moderate_time_pressure_ratio'),
                'high_time_pressure_ratio': LaunchConfiguration('high_time_pressure_ratio'),
            }
        ],
    )

    return LaunchDescription(launch_args + [node])
