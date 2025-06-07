from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav_rover_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='karimata.sora.r4@dc.tohoku.ac.jp',
    description='ARES Project rover nav2 control tutorial',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logged_waypoint_follower = nav_rover_control.logged_waypoint_follower:main',
            'interactive_waypoint_follower = nav_rover_control.interactive_waypoint_follower:main',
            'rover_control = nav_rover_control.rover_control:main',
            'gps_waypoint_logger = nav_rover_control.gps_waypoint_logger:main'
        ],
    },
)
