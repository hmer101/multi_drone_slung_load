import os, sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 launch swarm_load_carry gcs.launch.py"',
            ]],
            shell=True
        )
    ])