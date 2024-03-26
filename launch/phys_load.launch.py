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
    
    min_on_gcs = params["/**"]["ros__parameters"]["min_on_gcs"]
    
    ## INCLUDE LAUNCH FILES       
    load = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/load.launch.py']),
      launch_arguments={'env': 'phys'}.items()
      )

    # GCS background so can be on physical drone network if selected (more reliable than GCS)    
    gcs_background = ExecuteProcess(
        cmd=[[
            f'bash -c "ros2 run swarm_load_carry gcs_background --ros-args -r __node:=gcs_background1 --params-file {config}"', #gnome-terminal --tab -- -r __ns:=/gcs_1
        ]],
        shell=True
    )

    ## BUILD LAUNCH DESCRIPTION
    launch_description = []

    launch_description.append(load)
    
    # Only launch gcs_background if not launching on gcs, otherwise just launch load
    if min_on_gcs:
        launch_description.append(gcs_background)


    ## RUN LAUNCH FILES
    return LaunchDescription(launch_description)
