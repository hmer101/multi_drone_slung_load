import os, yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

ENV='phys'

def generate_launch_description():
    ## Get parameters 
    config = os.path.join(
      get_package_share_directory('swarm_load_carry'),
      'config',
      f'{ENV}.yaml'
      )

    with open(config, 'r') as file:
        params = yaml.safe_load(file)
    
    load_id = params["/**"]["ros__parameters"]["load_id"]
    this_drone_num = params["/**"]["ros__parameters"]["this_drone_num"]
    first_drone_num = params["/**"]["ros__parameters"]["first_drone_num"]

    ## INCLUDE LAUNCH FILES
    drone = ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 launch swarm_load_carry phys_drone.launch.py"',
            ]],
            shell=True
        )
    
    load = ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 launch swarm_load_carry load.launch.py load_id:={load_id} env:={ENV}"',
            ]],
            shell=True
        )
    
    gcs_background = ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 run swarm_load_carry gcs_background --ros-args -r __node:=gcs_background1 --params-file {config}"', 
            ]],
            shell=True
        )
    
    gcs_user = ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 run swarm_load_carry gcs_user --ros-args -r __node:=gcs_user1 --params-file {config}"', 
            ]],
            shell=True
        )

    launch_description = [drone]

    # Only launch load and gcs_background on 1st drone (drones must be networked)
    if this_drone_num == first_drone_num:
        launch_description.append(load)
        launch_description.append(gcs_background)
        #launch_description.append(gcs_user)

    ## RUN LAUNCH FILES
    return LaunchDescription(launch_description)

