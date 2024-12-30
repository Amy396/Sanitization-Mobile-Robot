from setuptools import find_packages, setup

package_name = 'localization_sanitazation'

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
            'FSMcontroller_node = localization_sanitazation.FSMcontroller_node : main',
            'localization_node = localization_sanitazation.localization_node : main',
            'sanitization_node = localization_sanitazation.sanitization_node : main',
            'navigation_node = localization_sanitazation.navigation_node : main'
        ],
    },
)
