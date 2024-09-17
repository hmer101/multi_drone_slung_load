import os, yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
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
    
    num_cameras = params["/**"]["ros__parameters"]["num_cameras"]
    first_drone_num = params["/**"]["ros__parameters"]["first_drone_num"]
    
    run_load_node_on = params["/**"]["ros__parameters"]["run_load_node_on"]
    run_background_node_on = params["/**"]["ros__parameters"]["run_background_node_on"]
    run_user_node_on = params["/**"]["ros__parameters"]["run_user_node_on"]
    
    run_estimator_on = params["/**"]["ros__parameters"]["run_estimator_on"]

    use_load_pose_estimator = params["/**"]["ros__parameters"]["use_load_pose_estimator"]
    gt_source = params["/**"]["ros__parameters"]["gt_source"]

    ## INCLUDE LAUNCH FILES
    # Launch drone
    drone = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('multi_drone_slung_load'), 'launch'),
         '/drone.launch.py']),
      launch_arguments={'env': 'phys', 'drone_id': str(drone_id_env)}.items()
    )

    # Camera
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('multi_drone_slung_load'), 'launch'),
            '/camera_phys.launch.py']),
        launch_arguments={'drone_id': str(drone_id_env)}.items()
        )
    
    # Visual pose measurement
    pose_measurement_visual = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('slung_pose_measurement'), 'launch'),
                '/visual_measurement.launch.py']),
            launch_arguments={'drone_id': str(drone_id_env), 'env': 'phys'}.items()
            )

    # Load, estimator, GCS user and GCS background so can be on physical drone network if selected (more reliable than GCS)
    load = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('multi_drone_slung_load'), 'launch'),
         '/load.launch.py']),
      launch_arguments={'env': 'phys'}.items()
      )

    gcs_user = ExecuteProcess(
            cmd=[[
                f'bash -c "ros2 run multi_drone_slung_load gcs_user --ros-args -r __node:=gcs_user1 --params-file {config}"', #-r __ns:=/gcs_1 
            ]],
            shell=True
        )

    gcs_background = ExecuteProcess(
            cmd=[[
                f'bash -c "ros2 run multi_drone_slung_load gcs_background --ros-args -r __node:=gcs_background1 --params-file {config}"', #gnome-terminal --tab --  -r __ns:=/gcs_1
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

    # Only launch load, estimator, gcs_background and/or gcs_user on 1st drone if set to do so
    if int(drone_id_env) == first_drone_num:
        # Launch load node if required
        if run_load_node_on == "drone":
            launch_description.append(load)
        
        # Launch estimator if required
        if use_load_pose_estimator and run_estimator_on == "drone":    
            estimator = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slung_pose_estimation'), 'launch'),
                    '/estimator_online.launch.py']),
                launch_arguments={'load_id': str(drone_id_env), 'env': 'phys'}.items()
                ) #TODO: fix load_id and drone_id mismatch. don't use load_id as param??
            
            launch_description.append(estimator)

        # Launch GCS background or user nodes are required
        if run_background_node_on == "drone":
            launch_description.append(gcs_background)

        if run_user_node_on == "drone":
            launch_description.append(gcs_user)

    # If using motion capture, launch the mocap conversion node
    if gt_source == "mocap":
        mocap_to_px4 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('highbay_vicon_px4'), 'launch'),
                '/highbay_to_px4.launch.py']),
            launch_arguments={'drone_id': str(drone_id_env)}.items()
            )
            
        launch_description.append(mocap_to_px4)


    ## RUN LAUNCH FILES and start MicroDDS agent
    return LaunchDescription(launch_description)
