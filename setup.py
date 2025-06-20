from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'waymo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'traffic_signs'), glob('waymo/traffic_signs/*.png')),
    ],
    # Entferne tf-transformations, füge scipy hinzu (wird oft als Teil von numpy installiert, aber sicher ist sicher)
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='fabian',
    maintainer_email='fabian.zaenker@web.de',
    description='ROS2 package for lane following, obstacle detection and passing.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detection_node = waymo.lane_detection_node:main',
            'gui_debug_node = waymo.gui_debug_node:main',
            'obstacle_detection_node = waymo.obstacle_detection_node:main',
            'state_manager_node = waymo.state_manager_node:main',
            'passing_obstacle_node = waymo.passing_obstacle_node:main',
            'traffic_light_detection_node = waymo.traffic_light_detection_node:main',
            'keyboard_handler_node = waymo.keyboard_handler_node:main',
            'sign_detection_node = waymo.sign_detection_node:main',
            'parking_node = waymo.parking_node:main',
            'reflection_filter = waymo.reflection_filter:main',
            'speed_governor_node = waymo.speed_governor_node:main',
            'intersection_handling_node = waymo.intersection_handling_node:main',
        ],
    },
)