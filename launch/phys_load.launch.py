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
    
    evaluate = params["/**"]["ros__parameters"]["evaluate"]
    load_pose_type = params["/**"]["ros__parameters"]["load_pose_type"]
    run_load_node_on = params["/**"]["ros__parameters"]["run_load_node_on"]
    run_background_node_on = params["/**"]["ros__parameters"]["run_background_node_on"]
    run_user_node_on = params["/**"]["ros__parameters"]["run_user_node_on"]

    ## INCLUDE LAUNCH FILES       
    load = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/load.launch.py']),
      launch_arguments={'env': 'phys'}.items()
      )

    # GCS user and background so can be on physical drone network if selected (more reliable than GCS)    
    gcs_user = ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 run swarm_load_carry gcs_user --ros-args -r __node:=gcs_user1 --params-file {config}"', #-r __ns:=/gcs_1 
            ]],
            shell=True
        )
    
    gcs_background = ExecuteProcess(
        cmd=[[
            f'bash -c "ros2 run swarm_load_carry gcs_background --ros-args -r __node:=gcs_background1 --params-file {config}"', #gnome-terminal --tab -- -r __ns:=/gcs_1
        ]],
        shell=True
    )

    # MicroXCREAgent - allows ROS2 communications from Pixhawk to be received if GPS is required
    micro_xcre_agent =  ExecuteProcess(
        cmd=[[
            f'bash -c "MicroXRCEAgent udp4 -p 8888"', #gnome-terminal --tab -- 
        ]],
        shell=True
    )

    ## BUILD LAUNCH DESCRIPTION
    launch_description = []

    # Launch load node if required
    if run_load_node_on == "load":
        launch_description.append(load)
    
    # Launch GCS background node if required
    if run_background_node_on == "load":
        launch_description.append(gcs_background)
    
    # Launch GCS user node if required
    if run_user_node_on == "load":
        launch_description.append(gcs_user)

    # Only need to communicate with the Pixhawk if GPS position of the load is required (for feedback or evaluation)
    if evaluate or load_pose_type == "ground_truth":
        launch_description.append(micro_xcre_agent)


    ## RUN LAUNCH FILES
    return LaunchDescription(launch_description)
