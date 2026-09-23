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
            'mission_manager = autorace_core.mission_manager:main',
        ],
    },
)
