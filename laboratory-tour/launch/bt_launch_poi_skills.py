import json
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import launch.logging

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'json_file', 
            default_value='',
            description='Path to the JSON file containing active tour POIs'
        ),
        DeclareLaunchArgument(
            'tour_name', 
            default_value='',
            description='Name of the tour to load from the JSON file'
        ),
        OpaqueFunction(function=launch_nodes_from_json)
    ])

def launch_nodes_from_json(context, *args, **kwargs):
    logger = launch.logging.get_logger('launch_nodes_from_json')

    json_file_path = os.path.expanduser(
        launch.substitutions.LaunchConfiguration('json_file').perform(context)
    )
    tour_name = launch.substitutions.LaunchConfiguration('tour_name').perform(context)
    
    if not json_file_path:
        logger.error("Missing 'json_file' argument.")
        return []
    if not tour_name:
        logger.error("Missing 'tour_name' argument.")
        return []
    
    if not os.path.isfile(json_file_path):
        logger.error(f"The JSON file '{json_file_path}' does not exist.")
        return []

    # Load JSON file
    try:
        with open(json_file_path, 'r') as f:
            tour_data = json.load(f)
    except Exception as e:
        logger.error(f"Failed to load JSON file '{json_file_path}'. Exception: {e}")
        return []

    # Get m_activeTourPoIs for the specified tour
    active_pois = tour_data.get(tour_name, {}).get("m_activeTourPoIs", [])
    
    # Check if there are active POIs
    if not active_pois:
        logger.error(f"No active POIs found for tour '{tour_name}' in {json_file_path}")
        return []

    # Create nodes for each active POI
    nodes = []
    for i, poi in enumerate(active_pois, start=0):
        nodes.append(Node(
            package='is_poi_done_skill',
            executable='is_poi_done_skill',
            name=f'is_poi_done_skill_{i}', 
            arguments=[str(i)] 
        ))
        nodes.append(Node(
            package='set_poi_skill',
            executable='set_poi_skill',
            name=f'set_poi_skill_{i}',
            arguments=[str(i)] 
        ))
    
    return nodes
