from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'autorace_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sxx',
    maintainer_email='wjrhee0610@g.kmou.ac.kr',
    description='Mission state machine, command arbitration and mission behaviors for AutoRace 2023.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_vel_mux = autorace_core.cmd_vel_mux:main',
            'construction_mission = autorace_core.construction_mission:main',
            'intersection_mission = autorace_core.intersection_mission:main',
            'lane_controller = autorace_core.lane_controller:main',
            'level_crossing_mission = autorace_core.level_crossing_mission:main',
            'mission_manager = autorace_core.mission_manager:main',
            'parking_mission = autorace_core.parking_mission:main',
            'traffic_light_mission = autorace_core.traffic_light_mission:main',
            'tunnel_mission = autorace_core.tunnel_mission:main',
        ],
    },
)
