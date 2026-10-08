from setuptools import find_packages
from setuptools import setup

package_name = 'autorace_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sxx',
    maintainer_email='wjrhee0610@g.kmou.ac.kr',
    description='Perception nodes for AutoRace 2023.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bev_projector = autorace_perception.bev_projector:main',
            'detect_lane = autorace_perception.detect_lane:main',
            'detect_level_crossing = autorace_perception.detect_level_crossing:main',
            'detect_sign = autorace_perception.detect_sign:main',
            'detect_stop_line = autorace_perception.detect_stop_line:main',
            'detect_traffic_light = autorace_perception.detect_traffic_light:main',
            'usb_camera = autorace_perception.usb_camera:main',
            'web_view = autorace_perception.web_view:main',
        ],
    },
)
