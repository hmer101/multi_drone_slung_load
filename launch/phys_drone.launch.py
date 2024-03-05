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

    ## INCLUDE LAUNCH FILES
    # Launch drone
    drone = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/drone.launch.py']),
      launch_arguments={'env': 'phys', 'drone_id': str(drone_id_env)}.items()
    )

    # Add microDDS agent
    launch_description = [   
        drone,
        ExecuteProcess(
            cmd=[[
                f'bash -c "MicroXRCEAgent udp4 -p 8888"', #gnome-terminal --tab -- 
            ]],
            shell=True
    )]

    # Launch camera and visual measurement if set to do so
    if drone_id_env <= int(num_cameras):
        # Camera
        camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('swarm_load_carry'), 'launch'),
                '/camera_phys.launch.py']),
            launch_arguments={'drone_id': str(drone_id_env)}.items()
            )

        launch_description.append(camera)

        # Visual pose measurement
        #if load_pose_type == "visual":
        pose_measurement_visual = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('slung_pose_estimation'), 'launch'),
                '/visual_measurement.launch.py']),
            launch_arguments={'drone_id': str(drone_id_env), 'env': 'phys'}.items()
            )
        
        launch_description.append(pose_measurement_visual)

    ## RUN LAUNCH FILES and start MicroDDS agent
    return LaunchDescription(launch_description)
