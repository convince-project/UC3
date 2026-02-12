from setuptools import find_packages, setup

package_name = 'planner_component'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools>61.0'],
    zip_safe=True,
    maintainer='ste',
    maintainer_email='stefano.bernagozzi@iit.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_component = planner_component.PlannerComponent:main'
        ],
    },
)
