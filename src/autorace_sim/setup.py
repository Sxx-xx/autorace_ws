from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'autorace_sim'


def model_files():
    """Install models/ recursively, keeping the directory layout gz expects."""
    entries = []
    for root, _, files in os.walk('models'):
        if files:
            entries.append(
                (os.path.join('share', package_name, root), [os.path.join(root, f) for f in files])
            )
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ] + model_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sxx',
    maintainer_email='wjrhee0610@g.kmou.ac.kr',
    description='AutoRace 2023 course for Gazebo Harmonic.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_traffic_light = autorace_sim.sim_traffic_light:main',
            'sim_level_crossing = autorace_sim.sim_level_crossing:main',
            'sim_firmware = autorace_sim.sim_firmware:main',
            'sim_clock = autorace_sim.sim_clock:main',
        ],
    },
)
