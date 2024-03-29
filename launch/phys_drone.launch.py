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
    run_load_node_on = params["/**"]["ros__parameters"]["run_load_node_on"]
    run_background_node_on = params["/**"]["ros__parameters"]["run_background_node_on"]

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
                f'bash -c "ros2 run swarm_load_carry gcs_background --ros-args -r __node:=gcs_background1 --params-file {config}"', #gnome-terminal --tab --  -r __ns:=/gcs_1
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

    # Only launch load and gcs_background on 1st drone if set to do so
    if int(drone_id_env) == first_drone_num:
        # Launch load node if required
        if run_load_node_on == "drone":
            launch_description.append(load)
        
        # Launch GCS background node if required
        if run_background_node_on == "drone":
            launch_description.append(gcs_background)


    ## RUN LAUNCH FILES and start MicroDDS agent
    return LaunchDescription(launch_description)
