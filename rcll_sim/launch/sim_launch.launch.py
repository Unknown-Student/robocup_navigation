#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

package_name = 'robocup_navigation'
world_file = 'rcll_sim/worlds/rcll-2027-default.world'

default_rviz_config_path = os.path.join(package_name, 'rviz/rviz_basic_settings.rviz')

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_radu_simulation = get_package_share_directory(package_name)

    # launch Gazebo by including its definition
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # load the world file
    verbose = DeclareLaunchArgument(
          'verbose',
          default_value= 'True')    
    
    world_arg = DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_radu_simulation, 'worlds', world_file), ''],
          description='SDF world file')

    return LaunchDescription([
        gazebo,
        verbose,
        world_arg
    ])