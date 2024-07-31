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
      get_package_share_directory('multi_drone_slung_load'),
      'config',
      'phys.yaml'
      )

    with open(config, 'r') as file:
        params = yaml.safe_load(file)
    
    # min_on_gcs = params["/**"]["ros__parameters"]["min_on_gcs"]
    # real_load_attached = params["/**"]["ros__parameters"]["real_load_attached"]
    run_load_node_on = params["/**"]["ros__parameters"]["run_load_node_on"]
    run_background_node_on = params["/**"]["ros__parameters"]["run_background_node_on"]
    run_user_node_on = params["/**"]["ros__parameters"]["run_user_node_on"]
    
    ## INCLUDE LAUNCH FILES    
    gcs = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('multi_drone_slung_load'), 'launch'),
         '/gcs.launch.py']),
      launch_arguments={'env': 'phys'}.items()
      )
    
    load = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('multi_drone_slung_load'), 'launch'),
         '/load.launch.py']),
      launch_arguments={'env': 'phys'}.items()
      )

    # Load and GCS background so can be on physical drone network if selected (more reliable than GCS)    
    gcs_user = ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 run multi_drone_slung_load gcs_user --ros-args -r __node:=gcs_user1 --params-file {config}"', #-r __ns:=/gcs_1 
            ]],
            shell=True
        )

    gcs_background = ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 run multi_drone_slung_load gcs_background --ros-args -r __node:=gcs_background1 --params-file {config}"', #gnome-terminal --tab -- -r __ns:=/gcs_1 
            ]],
            shell=True
        )

    ## BUILD LAUNCH DESCRIPTION
    launch_description = []

    # Launch load node if required
    if run_load_node_on == "gcs":
        launch_description.append(load)

    # Launch GCS background and user nodes if required
    # if run_background_node_on == "gcs" and run_user_node_on == "gcs":
    #     launch_description.append(gcs)
    if run_background_node_on == "gcs":
        launch_description.append(gcs_background)
    if run_user_node_on == "gcs":
        launch_description.append(gcs_user)

    ## RUN LAUNCH FILES
    return LaunchDescription(launch_description)
