from setuptools import setup
import os
from glob import glob

package_name = 'go'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='A package for tracking robot position and navigating back to starting point',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_tracker = go.position_tracker:main',
            'return_home = go.return_home_simple:main',
            'path_rec = go.path_rec:main',
            'rev_waypoint = go.rev_waypoint:main',
        ],
    },
)