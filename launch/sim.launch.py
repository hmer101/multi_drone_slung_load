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
      'sim.yaml'
      )

    with open(config, 'r') as file:
        params = yaml.safe_load(file)
    
    num_drones = params["/**"]["ros__parameters"]["num_drones"]
    first_drone_num = params["/**"]["ros__parameters"]["first_drone_num"]
    num_cameras = params["/**"]["ros__parameters"]["num_cameras"]
    evaluate = params["/**"]["ros__parameters"]["evaluate"]
    load_pose_type = params["/**"]["ros__parameters"]["load_pose_type"]

    ## INCLUDE ALL POSSIBLE LAUNCH TASKS
    # drones = IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource([os.path.join(
    #      get_package_share_directory('swarm_load_carry'), 'launch'),
    #      '/drones.launch.py'])
    #   )

    load = ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 launch swarm_load_carry load.launch.py load_id:={1} env:=sim"',
            ]],
            shell=True
        )
    
    gcs = ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 launch swarm_load_carry gcs.launch.py env:=sim"',
            ]],
            shell=True
        )
    
    slung_pose_estimation_visual = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('slung_pose_estimation'), 'launch'),
         '/visual_measurement_multi.launch.py'])
      ) #'/estimator.launch.py' launch_arguments={'env': 'sim'}.items()
    
    gz_bridge = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/gz_bridge.launch.py'])
      )

    # image_bridge = IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource([os.path.join(
    #      get_package_share_directory('swarm_load_carry'), 'launch'),
    #      '/image_bridge.launch.py'])
    #   )
    
    # gz_bridge = IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource([os.path.join(
    #      get_package_share_directory('swarm_load_carry'), 'launch'),
    #      '/gz_bridge_gt.launch.py'])
    #   )
    
    # gz_bridge_clock = IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource([os.path.join(
    #      get_package_share_directory('swarm_load_carry'), 'launch'),
    #      '/gz_bridge_clock.launch.py'])
    #     )
    

    ## COMPILE LAUNCH DESCRIPTION FROM SELECTED COMPONENTS (alter sim.yaml to change which components are included)
    launch_description = []
    
    # Drones
    # Launch each drone in a new terminal by calling the single drone launch file multiple times in new terminals
    for i in range(first_drone_num, num_drones+first_drone_num):
        launch_description.append(ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 launch swarm_load_carry drone.launch.py drone_id:={i} env:=sim"',
            ]],
            shell=True
        ))

    # Load and GCS
    launch_description.append(load)
    launch_description.append(gcs)


     # Slung load pose estimation
    if num_cameras > 0: #and load_pose_type == "visual": 
      launch_description.append(slung_pose_estimation_visual)


    # Gazebo parameter and image bridges
    launch_description.append(gz_bridge)

    # # Ground truth - required for evaluation
    # if evaluate:
    #   launch_description.append(gz_bridge)

    # # Clock bridge for ground truth and camera info
    # launch_description.append(gz_bridge_clock)


    ## RUN LAUNCH FILES
    return LaunchDescription(launch_description)