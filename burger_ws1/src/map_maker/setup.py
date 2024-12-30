from setuptools import find_packages, setup

package_name = 'map_maker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amy',
    maintainer_email='amy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planning_navigator_node = map_maker.path_planning_navigator_node : main' ,
            'frontier_explorer_node = map_maker.frontier_explorer_node : main',
            'rotation_node = map_maker.rotation_node : main',
            'FSM_controller_node = map_maker.FSM_controller_node : main'
        ],
    },
)
