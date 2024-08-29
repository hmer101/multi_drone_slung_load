import os, yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
     ## GET PARAMETERS
    config = os.path.join(
      get_package_share_directory('multi_drone_slung_load'),
      'config',
      'sim.yaml'
      )

    with open(config, 'r') as file:
        params = yaml.safe_load(file)
    
    num_drones = params["/**"]["ros__parameters"]["num_drones"]
    first_drone_num = params["/**"]["ros__parameters"]["first_drone_num"]
    num_cameras = params["/**"]["ros__parameters"]["num_cameras"]
    run_logger = params["/**"]["ros__parameters"]["run_logger"]
    logging_rosbag = params["/**"]["ros__parameters"]["logging_rosbag"]
    

    ## INCLUDE ALL POSSIBLE LAUNCH TASKS
    load = ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 launch multi_drone_slung_load load.launch.py load_id:={1} env:=sim"',
            ]],
            shell=True
        )
    
    gcs = ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 launch multi_drone_slung_load gcs.launch.py env:=sim"',
            ]],
            shell=True
        )
    
    gz_bridge = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('multi_drone_slung_load'), 'launch'),
         '/gz_bridge.launch.py'])
      )  
    
    log_rosbag = ExecuteProcess(
            cmd=['ros2', 'launch', 'multi_drone_slung_load', 'logging.launch.py', 'env:=sim'],
            output='screen'
        )

    logger = ExecuteProcess(
            cmd=['ros2', 'launch', 'slung_pose_estimation', 'logger.launch.py', 'env:=sim'],
            output='screen'
        )


    ## COMPILE LAUNCH DESCRIPTION FROM SELECTED COMPONENTS (alter sim.yaml to change which components are included)
    launch_description = []
    
    # Drones
    # Launch each drone in a new terminal by calling the single drone launch file multiple times in new terminals
    for i in range(first_drone_num, num_drones+first_drone_num):
        launch_description.append(ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 launch multi_drone_slung_load drone.launch.py drone_id:={i} env:=sim"',
            ]],
            shell=True
        ))

    # Slung load pose estimation
    # if num_cameras > 0: #and load_pose_type == "visual": 
    for i in range(first_drone_num, num_cameras+first_drone_num):
        launch_description.append(ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 launch slung_pose_estimation visual_measurement.launch.py drone_id:={i} env:=sim"',
            ]],
            shell=True
        ))

    # Load and GCS
    launch_description.append(load)
    launch_description.append(gcs)

    # Gazebo parameter and image bridges
    launch_description.append(gz_bridge)

    # Logging
    if run_logger:
        launch_description.append(logger)

    if logging_rosbag:
        launch_description.append(log_rosbag)


    ## RUN LAUNCH FILES
    return LaunchDescription(launch_description)