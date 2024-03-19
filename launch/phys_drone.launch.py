import os, yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
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
    
    num_cameras = params["/**"]["ros__parameters"]["num_cameras"]
    first_drone_num = params["/**"]["ros__parameters"]["first_drone_num"]
    min_on_gcs = params["/**"]["ros__parameters"]["min_on_gcs"]
    real_load_attached = params["/**"]["ros__parameters"]["real_load_attached"]

    ## INCLUDE LAUNCH FILES
    # Launch drone
    drone = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/drone.launch.py']),
      launch_arguments={'env': 'phys', 'drone_id': str(drone_id_env)}.items()
    )

    # Camera
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('swarm_load_carry'), 'launch'),
            '/camera_phys.launch.py']),
        launch_arguments={'drone_id': str(drone_id_env)}.items()
        )
    
    # Visual pose measurement
    pose_measurement_visual = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('slung_pose_estimation'), 'launch'),
                '/visual_measurement.launch.py']),
            launch_arguments={'drone_id': str(drone_id_env), 'env': 'phys'}.items()
            )

    # Load and GCS background so can be on physical drone network if selected (more reliable than GCS)
    load = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/load.launch.py']),
      launch_arguments={'env': 'phys'}.items()
      )
    
    gcs_background = ExecuteProcess(
            cmd=[[
                f'bash -c "ros2 run swarm_load_carry gcs_background --ros-args -r __node:=gcs_background1 --params-file {config}"', #gnome-terminal --tab -- 
            ]],
            shell=True
        )

    ## BUILD LAUNCH DESCRIPTION
    # Add drone node and microDDS agent
    launch_description = [   
        drone,
        ExecuteProcess(
            cmd=[[
                f'bash -c "MicroXRCEAgent udp4 -p 8888"', #gnome-terminal --tab -- 
            ]],
            shell=True
    )]
    
    # Launch camera and visual measurement if set to do so
    if int(drone_id_env) <= int(num_cameras):       
        launch_description.append(camera)

        #if load_pose_type == "visual":
        launch_description.append(pose_measurement_visual)

    # Only launch load and gcs_background on 1st drone if set to do so i.e. not running on GCS or load (drones must be networked)
    if min_on_gcs and not real_load_attached and int(drone_id_env) == first_drone_num:
        launch_description.append(load)
        launch_description.append(gcs_background)

    ## RUN LAUNCH FILES and start MicroDDS agent
    return LaunchDescription(launch_description)
