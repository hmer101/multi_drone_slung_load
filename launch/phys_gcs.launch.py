import os, yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ## GET PARAMETERS
    drone_id_env = os.environ.get('DRONE_ID', 1) # Note must first set environment variable with export DRONE_ID=1

    config = os.path.join(
      get_package_share_directory('swarm_load_carry'),
      'config',
      'phys.yaml'
      )

    with open(config, 'r') as file:
        params = yaml.safe_load(file)
    
    min_on_gcs = params["/**"]["ros__parameters"]["min_on_gcs"]
    real_load_attached = params["/**"]["ros__parameters"]["real_load_attached"]
    
    ## INCLUDE LAUNCH FILES    
    gcs = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/gcs.launch.py']),
      launch_arguments={'env': 'phys'}.items()
      )
    
    load = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/load.launch.py']),
      launch_arguments={'env': 'phys'}.items()
      )

    # Load and GCS background so can be on physical drone network if selected (more reliable than GCS)    
    gcs_user = ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 run swarm_load_carry gcs_user --ros-args -r __node:=gcs_user1 --params-file {config}"', 
            ]],
            shell=True
        )

    ## BUILD LAUNCH DESCRIPTION
    launch_description = []

    # Only launch load and gcs_background if not launching on drones, otherwise just launch gcs_user
    if not min_on_gcs:
        launch_description.append(gcs)

        if not real_load_attached:
            launch_description.append(load)

    else:
        launch_description.append(gcs_user)

    ## RUN LAUNCH FILES
    return LaunchDescription(launch_description)
