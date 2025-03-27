import xml.etree.ElementTree as ET
import argparse
import subprocess

def create_symlinks_from_xml(xml_file):
    tree = ET.parse(xml_file)
    root = tree.getroot()
    
    for module in root.findall('module'):
        name_elem = module.find('name')
        parameters = module.find('parameters').text
        if name_elem is not None and name_elem.text.startswith("ros2") and parameters:
            params_split = parameters.split()
            if len(params_split) > 1:
                symlink_name = f"{name_elem.text}"
                target = "$(which ros2)"
                # symlink = symlink_name
                symlink = f"/usr/local/bin/{symlink_name}"
                command = f"sudo ln -s {target} {symlink}"
                print(command)
                try:
                    subprocess.run(command, shell=True, check=True)
                except subprocess.CalledProcessError as e:
                    print(f"Error creating symlink {symlink}: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate and execute symbolic links from an XML file if the module name is 'ros2'.")
    parser.add_argument('xml_file', type=str, help="The path to the XML file.")

    args = parser.parse_args()
    create_symlinks_from_xml(args.xml_file)
