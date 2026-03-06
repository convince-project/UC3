#!/usr/bin/env python3
"""
Merge two BehaviorTree XML files into a single file.
Replaces the PoiScheduler subtree in bt_poi_complex.xml with the one from bt_scheduler.xml.
Creates a single flattened BehaviorTree by inlining all SubTrees.
"""

import xml.etree.ElementTree as ET
import argparse
from pathlib import Path
import copy


def merge_bt_files(main_file, scheduler_file, output_file):
    """
    Merge two BT XML files.
    
    Args:
        main_file: Path to the main BT file (bt_poi_complex.xml)
        scheduler_file: Path to the scheduler BT file (bt_scheduler.xml)
        output_file: Path to save the merged output
    """
    # Parse both XML files (strip leading whitespace to fix XML declaration issues)
    print(f"Reading main file: {main_file}")
    with open(main_file, 'r', encoding='utf-8') as f:
        main_content = f.read().lstrip()
    main_tree = ET.ElementTree(ET.fromstring(main_content))
    main_root = main_tree.getroot()
    
    print(f"Reading scheduler file: {scheduler_file}")
    with open(scheduler_file, 'r', encoding='utf-8') as f:
        scheduler_content = f.read().lstrip()
    scheduler_tree = ET.ElementTree(ET.fromstring(scheduler_content))
    scheduler_root = scheduler_tree.getroot()
    
    # Find the PoiScheduler BehaviorTree in the scheduler file
    scheduler_poi = None
    for bt in scheduler_root.findall('BehaviorTree'):
        if bt.get('ID') == 'PoiScheduler':
            scheduler_poi = bt
            break
    
    if scheduler_poi is None:
        print("WARNING: PoiScheduler not found in scheduler file")
    else:
        print("Found PoiScheduler in scheduler file")
        
        # Find and replace the PoiScheduler in main file
        replaced = False
        for i, bt in enumerate(main_root.findall('BehaviorTree')):
            if bt.get('ID') == 'PoiScheduler':
                # Get the index in the root
                bt_index = list(main_root).index(bt)
                # Remove old PoiScheduler
                main_root.remove(bt)
                # Insert new PoiScheduler at the same position
                main_root.insert(bt_index, scheduler_poi)
                print("Replaced PoiScheduler in main file")
                replaced = True
                break
        
        if not replaced:
            print("WARNING: PoiScheduler not found in main file, appending it")
            # Find the position before TreeNodesModel
            insert_pos = len(list(main_root))
            for i, child in enumerate(main_root):
                if child.tag == 'TreeNodesModel':
                    insert_pos = i
                    break
            main_root.insert(insert_pos, scheduler_poi)
    
    # Write the merged output
    print(f"Writing merged output to: {output_file}")
    
    # Flatten the tree - inline all SubTrees into the main BehaviorTree
    print("Flattening BehaviorTree structure...")
    flatten_behavior_tree(main_root)
    
    # Add service_name attributes to ROS2Action and ROS2Condition nodes
    print("Adding service_name attributes...")
    add_service_names(main_root)
    
    # Convert SequenceStar to SequenceWithMemory
    print("Converting SequenceStar to SequenceWithMemory...")
    convert_sequence_star_to_with_memory(main_root)
    
    # Remove TreeNodesModel
    print("Removing TreeNodesModel...")
    for elem in list(main_root.findall('TreeNodesModel')):
        main_root.remove(elem)
    
    # Pretty print with proper indentation
    indent_xml(main_root)
    
    # Create the output tree
    output_tree = ET.ElementTree(main_root)
    
    # Write to file with XML declaration
    with open(output_file, 'wb') as f:
        output_tree.write(f, encoding='UTF-8', xml_declaration=True)
    
    print(f"✓ Successfully merged files into {output_file}")
    
    # Print summary
    behavior_trees = [bt.get('ID') for bt in main_root.findall('BehaviorTree')]
    print(f"\nBehaviorTrees in merged file: {len(behavior_trees)}")
    print(f"Main tree to execute: {main_root.get('main_tree_to_execute')}")


def flatten_behavior_tree(root):
    """
    Flatten the BehaviorTree by inlining all SubTrees into the main tree.
    Removes all BehaviorTree definitions except the main one.
    """
    # Get the main tree ID
    main_tree_id = root.get('main_tree_to_execute')
    
    # Build a dictionary of all BehaviorTree definitions
    bt_definitions = {}
    for bt in root.findall('BehaviorTree'):
        bt_id = bt.get('ID')
        bt_definitions[bt_id] = bt
    
    # Find the main BehaviorTree
    main_bt = bt_definitions.get(main_tree_id)
    if main_bt is None:
        print(f"WARNING: Main BehaviorTree '{main_tree_id}' not found")
        return
    
    # Recursively inline all SubTree references in the main tree
    inline_subtrees(main_bt, bt_definitions, visited=set())
    
    # Remove all BehaviorTree elements
    for bt in list(root.findall('BehaviorTree')):
        root.remove(bt)
    
    # Add back only the main (now flattened) BehaviorTree
    # Insert it at the beginning (before TreeNodesModel)
    root.insert(0, main_bt)


def inline_subtrees(elem, bt_definitions, visited, depth=0):
    """
    Recursively replace all SubTree elements with their actual content.
    """
    max_depth = 20  # Prevent infinite recursion
    if depth > max_depth:
        print(f"WARNING: Maximum recursion depth reached at depth {depth}")
        return
    
    # Process all children
    for i, child in enumerate(list(elem)):
        if child.tag == 'SubTree':
            subtree_id = child.get('ID')
            
            # Check for circular references
            if subtree_id in visited:
                print(f"WARNING: Circular reference detected for SubTree '{subtree_id}'")
                continue
            
            # Get the subtree definition
            subtree_def = bt_definitions.get(subtree_id)
            if subtree_def is None:
                print(f"WARNING: SubTree '{subtree_id}' definition not found")
                continue
            
            # Get the content of the subtree (first child element)
            subtree_content = list(subtree_def)
            if not subtree_content:
                print(f"WARNING: SubTree '{subtree_id}' is empty")
                elem.remove(child)
                continue
            
            # Create a deep copy of the subtree content
            subtree_root = copy.deepcopy(subtree_content[0])
            
            # Mark as visited for recursion
            new_visited = visited | {subtree_id}
            
            # Recursively inline any SubTrees within this subtree
            inline_subtrees(subtree_root, bt_definitions, new_visited, depth + 1)
            
            # Replace the SubTree element with the actual content
            parent = elem
            child_index = list(parent).index(child)
            parent.remove(child)
            parent.insert(child_index, subtree_root)
        else:
            # Recursively process this child
            inline_subtrees(child, bt_definitions, visited, depth)


def add_service_names(root):
    """
    Add service_name attributes to ROS2Action and ROS2Condition elements.
    - For ROS2Condition: add service_name="/NameSkill/tick"
    - For ROS2Action: add service_name_tick="/NameSkill/tick" and service_name_halt="/NameSkill/halt"
    """
    def process_element(elem):
        """Recursively process all elements in the tree."""
        # Process current element
        if elem.tag == 'ROS2Condition':
            name = elem.get('name')
            if name and 'service_name' not in elem.attrib:
                service_name = f"/{name}Skill/tick"
                elem.set('service_name', service_name)
        
        elif elem.tag in ['ROS2Action', 'Action']:
            name = elem.get('name')
            if name:
                if 'service_name_tick' not in elem.attrib:
                    service_name_tick = f"/{name}Skill/tick"
                    elem.set('service_name_tick', service_name_tick)
                if 'service_name_halt' not in elem.attrib:
                    service_name_halt = f"/{name}Skill/halt"
                    elem.set('service_name_halt', service_name_halt)
        
        elif elem.tag == 'Condition':
            name = elem.get('name')
            if name and 'service_name' not in elem.attrib:
                service_name = f"/{name}Skill/tick"
                elem.set('service_name', service_name)
        
        # Recursively process children
        for child in elem:
            process_element(child)
    
    # Start processing from root
    process_element(root)


def convert_sequence_star_to_with_memory(root):
    """
    Convert all SequenceStar elements to SequenceWithMemory.
    """
    def process_element(elem):
        """Recursively process all elements in the tree."""
        # Convert SequenceStar tags
        if elem.tag == 'SequenceStar':
            elem.tag = 'SequenceWithMemory'
        
        # Recursively process children
        for child in list(elem):
            process_element(child)
    
    # Start processing from root
    process_element(root)


def indent_xml(elem, level=0):
    """Add proper indentation to XML for pretty printing."""
    indent = "\n" + "  " * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = indent + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = indent
        for child in elem:
            indent_xml(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = indent
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = indent


def main():
    parser = argparse.ArgumentParser(
        description='Merge two BehaviorTree XML files into a single file'
    )
    parser.add_argument(
        '--main',
        default='src/behavior_tree/BT/bt_poi_complex.xml',
        help='Path to main BT file (default: src/behavior_tree/BT/bt_poi_complex.xml)'
    )
    parser.add_argument(
        '--scheduler',
        default='src/behavior_tree/BT/bt_scheduler.xml',
        help='Path to scheduler BT file (default: src/behavior_tree/BT/bt_scheduler.xml)'
    )
    parser.add_argument(
        '--output',
        default='model-high-level/Policy/bt_merged.xml',
        help='Path to output merged file (default: src/behavior_tree/BT/bt_merged.xml)'
    )
    
    args = parser.parse_args()
    
    # Check if input files exist
    main_path = Path(args.main)
    scheduler_path = Path(args.scheduler)
    
    if not main_path.exists():
        print(f"ERROR: Main file not found: {main_path}")
        return 1
    
    if not scheduler_path.exists():
        print(f"ERROR: Scheduler file not found: {scheduler_path}")
        return 1
    
    # Create output directory if needed
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Perform the merge
    try:
        merge_bt_files(main_path, scheduler_path, output_path)
        return 0
    except Exception as e:
        print(f"ERROR: Failed to merge files: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    exit(main())
