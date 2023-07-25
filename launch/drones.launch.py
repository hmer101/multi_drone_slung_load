import os, sys, yaml

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ## GET PARAMETERS
    config = os.path.join(
      get_package_share_directory('swarm_load_carry'),
      'config',
      'sim.yaml'
      )

    with open(config, 'r') as file:
        params = yaml.safe_load(file)
    
    num_drones = params["/**"]["ros__parameters"]["num_drones"]
    first_drone_num = params["/**"]["ros__parameters"]["first_drone_num"]

    ## LAUNCH BODY
    # Launch each drone in a new terminal by calling the single drone launch file multiple times in new terminals
    launch_description = []

    for i in range(first_drone_num, num_drones+first_drone_num):
        launch_description.append(ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 launch swarm_load_carry drone.launch.py drone_id:={i} env:=sim"',
            ]],
            shell=True
        ))

    return LaunchDescription(launch_description)