import json
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import launch.logging
import xml.etree.ElementTree as ET

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

def generate_poi_scheduler_sub_bt(active_pois, bt_file_name):
    # Verify if the BT file exists
    if not os.path.isfile(bt_file_name):
        raise FileNotFoundError(f"The '{bt_file_name}' BT file does not exist.")

    # Read the BT file as XML
    tree = ET.parse(bt_file_name)
    root = tree.getroot()

    # Find the <BehaviorTree> node with ID="PoiScheduler"
    behavior_tree = root.find(".//BehaviorTree[@ID='PoiScheduler']")
    if behavior_tree is None:
        raise ValueError("The <BehaviorTree ID='PoiScheduler'> node has not been found in the BT file.")

    # Find the <Fallback> node inside <BehaviorTree>
    fallback_node = behavior_tree.find("./Fallback")
    if fallback_node is None:
        raise ValueError("The <Fallback> node has not been found in the BT file.")

    # Remove existing children of <Fallback> except for the <Action> node with name="Reset"
    for child in list(fallback_node):
        name_attr = child.attrib.get("name", "")
        if "Reset" in name_attr:
            name_attr = child.attrib.get("name", "")
            print(f"Found 'Reset' node")
            reset_node = child
        fallback_node.remove(child)

    # Add a <Sequence> node for each active POI
    for index, poi in enumerate(active_pois, start=0):
        # Add a <Sequence> node
        sequence_element = ET.Element("Sequence")

        # Add a <Inverter> node with a <Condition> child
        inverter_element = ET.SubElement(sequence_element, "Inverter")
        ET.SubElement(inverter_element, "Condition", {
            "ID": "ROS2Condition",
            "interface": "ROS2SERVICE",
            "isMonitored": "false",
            "name": f"IsPoiDone{index}"
        })

        # Add an <Action> node
        ET.SubElement(sequence_element, "Action", {
            "ID": "ROS2Action",
            "interface": "ROS2SERVICE",
            "isMonitored": "false",
            "name": f"SetPoi{index}"
        })

        # Add the <Sequence> node as a child of <Fallback>
        fallback_node.append(sequence_element)

    # If the "Reset" node exists, append it back to the end
    if reset_node is not None:
        fallback_node.append(reset_node)
        print(f"Appended 'Reset' node at the end of <Fallback>.")
    else:
        print("No 'Reset' node found; nothing to append.")

    ET.indent(behavior_tree, '    ', level=1)  
    # Save the updated XML
    tree.write(bt_file_name, encoding="utf-8", xml_declaration=True)

    print(f"The '{bt_file_name}' BT file has been updated with {len(active_pois)} POIs.")



def launch_nodes_from_json(context, *args, **kwargs):
    logger = launch.logging.get_logger('launch_nodes_from_json')
    
    json_file_path = os.path.expanduser(
        launch.substitutions.LaunchConfiguration('json_file').perform(context)
    )
    tour_name = launch.substitutions.LaunchConfiguration('tour_name').perform(context)
    if not json_file_path:
        
        print("Missing 'json_file' argument.")
        return []
    if not tour_name:
        print("Missing 'tour_name' argument.")
        return []
    
    if not os.path.isfile(json_file_path):
        print(f"The JSON file '{json_file_path}' does not exist.")
        return []

    # Load JSON file
    try:
        with open(json_file_path, 'r') as f:
            tour_data = json.load(f)
    except Exception as e:
        print(f"Failed to load JSON file '{json_file_path}'. Exception: {e}")
        return []

    # Get m_activeTourPoIs for the specified tour
    active_pois = tour_data.get(tour_name, {}).get("m_activeTourPoIs", [])
    
    # Check if there are active POIs
    if not active_pois:
        print(f"No active POIs found for tour '{tour_name}' in {json_file_path}")
        return []

    # Create nodes for each active POI
    nodes = []
    for i, poi in enumerate(active_pois, start=0):
        nodes.append(Node(
            package='is_poi_done_skill',
            executable=f'is_poi_done_skill_{i}',
            name=f'is_poi_done_skill_{i}', 
            arguments=[str(i)] 
        ))
        nodes.append(Node(
            package='set_poi_skill',
            executable=f'set_poi_skill_{i}',
            name=f'set_poi_skill_{i}',
            arguments=[str(i)] 
        ))

    bt_file_name = launch.substitutions.LaunchConfiguration('bt_file').perform(context)
    generate_poi_scheduler_sub_bt(active_pois, bt_file_name)
    
    return nodes
