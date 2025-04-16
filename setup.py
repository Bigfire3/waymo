from setuptools import find_packages, setup

package_name = 'waymo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/waymo_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabian',
    maintainer_email='fabian.zaenker@web.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detection_node = waymo.lane_detection_node:main',
            'gui_debug_node = waymo.gui_debug_node:main',
            'obstacle_detection_node = waymo.obstacle_detection_node:main',
            'state_manager_node = waymo.state_manager_node:main',
        ],
    },
)
