import os, yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ## GET PARAMETERS
    config = os.path.join(
      get_package_share_directory('swarm_load_carry'),
      'config',
      'phys.yaml'
      )

    with open(config, 'r') as file:
        params = yaml.safe_load(file)
    
    this_drone_num = params["/**"]["ros__parameters"]["this_drone_num"]

    ## INCLUDE LAUNCH FILES
    drone = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/drone.launch.py']),
      launch_arguments={'env': 'phys', 'drone_id': f'{this_drone_num}'}.items()
      )

    ## RUN LAUNCH FILES and start MicroDDS agent
    return LaunchDescription([
        drone,
        ExecuteProcess(
            cmd=[[
                f'bash -c "MicroXRCEAgent udp4 -p 8888"', #gnome-terminal --tab -- 
            ]],
            shell=True
        )])

# TODO: USE ENVIRONMENT VARS TO DEFINE DRONE NUM ON EACH RPI: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#environment-variables
