# Copyright 2024, Markus Buchholz

from setuptools import setup
import os
from glob import glob

package_name = 'move_blueboat'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include any other directories here
    ],
    install_requires=['setuptools', 'tf2_ros'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Your package description',
    license='Your license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Update this if you have any executables
            'robot_controller = move_blueboat.robot_controller:main',
            'run_blueboat_mission = move_blueboat.run_blueboat_mission:main',
            'run_mpc = move_blueboat.run_mpc_blueboat_mission:main',
            'run_horizion_mpc = move_blueboat.run_horizon_mpc_blueboat_mission:main',
            'thrust_allocator = move_blueboat.thrust_allocator:main',
            'run_mission = move_blueboat.run_mission:main',
            'dp_run = move_blueboat.dp_run:main',
            'thread_dp_run_with_heading = move_blueboat.threading_dp_run_with_heading:main',
            'watchdog = move_blueboat.watchdog:main',
            'key_controller = move_blueboat.key_controller:main',
            'gui_joy_controller = move_blueboat.gui_joy_controller:main',
            'joy_sim_controller = move_blueboat.joy_sim_controller:main',
            'mission_planner_blueboat = move_blueboat.mission_planner_blueboat:main',
            'way_controller_for_boat_mission = move_blueboat.way_controller_for_boat_mission:main',
            'pid_controller_for_boat_mission = move_blueboat.pid_controller_for_boat_mission:main',
            'mpc_controller_for_boat_mission = move_blueboat.mpc_controller_for_boat_mission:main',
            'dp_beacon_dvl_run_boat_waypoint = move_blueboat.dp_beacon_dvl_run_boat_waypoint:main',

        ],
    },
)
