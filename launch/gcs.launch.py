import os, sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess

DEFAULT_ENV='sim'

def generate_launch_description():
    ## LAUNCH ARGUMENTS
    launch_arg_sim_phys = DeclareLaunchArgument(
      'env', default_value=str(DEFAULT_ENV)
    )

    # Get arguments
    env = DEFAULT_ENV
    for arg in sys.argv:
        if arg.startswith("env:="):
            env = int(arg.split(":=")[1])

    ## GET PARAMETERS

    ## LAUNCH
    return LaunchDescription([
        ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 run swarm_load_carry gcs --ros-args --params-file ~/px4_ros_com_ros2/src/swarm_load_carry/config/{env}.yaml"',
            ]],
            shell=True
        )
    ])