#!/usr/bin/env python3
import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('mini_project'), 'launch')

    pkg_neato_sim = get_package_share_directory('neato2_gazebo')
    gauntlet_filename = os.path.join(pkg_neato_sim, 'launch', 'neato_gauntlet_world.py')
    gauntlet_launchfile = PythonLaunchDescriptionSource(gauntlet_filename)

    return LaunchDescription([
        IncludeLaunchDescription(gauntlet_launchfile),
    ])