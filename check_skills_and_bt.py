import sys
import xml.etree.ElementTree as ET
import re

def camel_to_snake(name):
    name = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', name).lower()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 check_skills_and_bt.py <path_to_bt_xml> <path_to_skill_application_xml>")
        sys.exit(1)
    bt_xml_path = sys.argv[1]
    skill_app_xml_path = sys.argv[2]
    print(f"Checking skills in behavior tree: {bt_xml_path} against skill application: {skill_app_xml_path}")

    # parse the xml of the bt and check if the skills are defined in the skill application xml

    bt_tree = ET.parse(bt_xml_path)
    bt_root = bt_tree.getroot()

    skill_app_tree = ET.parse(skill_app_xml_path)
    skill_app_root = skill_app_tree.getroot()

    skill_names_in_bt = set()
    print(bt_root)
    all_bt_nodes = bt_root.findall(".//ROS2Condition") + bt_root.findall(".//ROS2Action")
    print(f"Total behavior tree nodes found: {len(all_bt_nodes)}")
    for node in all_bt_nodes:
        # print(node)
        skill_name = node.get("name")
        print(f"Found skill in behavior tree: {skill_name}")
        skill_names_in_bt.add(skill_name)

    skill_names_in_app = set()
    for node in skill_app_root.findall("module"):
        param_node = node.find("parameters")
        if param_node is None:
            continue
        text_inside = param_node.text
        if text_inside is None:
            continue
        if "dummy_condition" in text_inside or "dummy_action" in text_inside:
            splitted = text_inside.split(" ")
            skill_name = splitted[4]
            skill_names_in_app.add(skill_name)
            print(f"Found skill in application: {skill_name}")
        else:
            if "run" in text_inside:
                splitted = text_inside.split(" ")
                skill_name = splitted[2][:-5].replace("_", " ").title().replace(" ", "")
                skill_names_in_app.add(skill_name)
                print(f"Found skill in application: {skill_name}")



    undefined_skills = skill_names_in_bt - skill_names_in_app
    if undefined_skills:
        print("The following skills are used in the behavior tree but not defined in the skill application:")
        for skill in undefined_skills:
            print(f" - {skill}")
    skills_not_used = skill_names_in_app - skill_names_in_bt
    if skills_not_used:
        print("The following skills are defined in the skill application but not used in the behavior tree:")
        for skill in skills_not_used:
            print(f" - {skill}")
    else:
        print("All skills used in the behavior tree are defined in the skill application.")