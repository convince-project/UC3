import re
import xml.etree.ElementTree as ET
import ros2pkg
import ros2pkg.api
import ros2pkg.resource
import ros2pkg.resource.ament_python
import ros2pkg.resource.package_environment

### This function is used to convert a string from Pascal/Camel case to snake case
# @param name: string to convert
# @return name in snake case
def convert_to_snake_case(name):
    # Convert Pascal case to snake case
    name = re.sub(r'(?<!^)(?=[A-Z])', '_', name).lower()
    return name

### This function is used to convert a skill name to the name of the executable
# @param name: name of the skill
# @return name of the executable/package
def toSkillName(name):
    name = convert_to_snake_case(name)+'_skill'
    return name

### This function is used to parse the xml file of a Behavior Tree and obtain a dictionary containing all the involved skills
# @param file_path: path to the xml file
# @param name_changer: function to change the name of the skill
# @return result: dictionary containing all the skills
def parse_xml_file(file_path, name_changer=None):
    tree = ET.parse(file_path)
    root = tree.getroot()

    result = {}

    for element in root.iter():
        if element.tag == 'Action' or element.tag == 'Condition':
            if 'name' in element.attrib:
                if name_changer is not None:
                    if callable(name_changer):
                        result[name_changer(element.attrib['name'])] = {"type": element.tag,
                                                                        "original_name": element.attrib['name']}
                else:
                    result[element.attrib['name']] = {"type": element.tag}

    return result

### This function is not used in the code, but it is useful to understand how to extract the nodes from the xml file
# @param bt_xml_path: path to the xml file
# @param exclude_list: list of packages to exclude
# @return existing_packages: list of existing packages
# @return non_existing_packages: list of non existing packages
def get_skills_nodes(bt_xml_path, exclude_list=[]):

    xml_output = parse_xml_file(bt_xml_path, toSkillName)
    existing_packages = []
    non_existing_packages = []

    for key, value in xml_output.items():
        package_name = key
        executable_name = key

        if package_exists(package_name) and package_name not in exclude_list:
            node = {
                'package_name': package_name,
                'executable_name': executable_name
            }
            existing_packages.append(node)
        else:
            skill_name = value['original_name'] if 'original_name' in value else key
            package_type = value['type']
            package_name = 'dummy_action' if package_type == 'Action' else 'dummy_condition'
            default_status = 'SUCCESS'

            non_existing_package = {
                'skill_name': skill_name,
                'package_name': package_name,
                'executable_name': package_name,
                'default_status': default_status
            }

            non_existing_packages.append(non_existing_package)

    return existing_packages, non_existing_packages

### This tells if a package exists in the workspace
# @param package_name: name of the package
# @return True if the package exists, False otherwise
def package_exists(package_name):
    # try:
    #     package_path = ros2pkg.get_package_share_directory(package_name)
    #     return True
    # except ros2pkg.api.PackageNotFoundError:
    #     return False
    return package_name in ros2pkg.api.get_package_names()
