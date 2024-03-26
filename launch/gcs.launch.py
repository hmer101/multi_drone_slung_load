import os, sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess

DEFAULT_ENV='phys'

def generate_launch_description():
    ## LAUNCH ARGUMENTS
    launch_arg_sim_phys = DeclareLaunchArgument(
      'env', default_value=str(DEFAULT_ENV)
    )

    # Get arguments
    env = DEFAULT_ENV
    for arg in sys.argv:
        if arg.startswith("env:="):
            env = str(arg.split(":=")[1])

    ## GET PARAMETERS
    config = None

    if env=="sim":
      config = os.path.join(
        get_package_share_directory('swarm_load_carry'),
        'config',
        'sim.yaml'
        )
    elif env=="phys":
       config = os.path.join(
        get_package_share_directory('swarm_load_carry'),
        'config',
        'phys.yaml'
        ) 


    ## LAUNCH
    return LaunchDescription([
        launch_arg_sim_phys,
        ExecuteProcess(
            cmd=[[
                f'bash -c "ros2 run swarm_load_carry gcs_background --ros-args -r __node:=gcs_background1 --params-file {config}"', #gnome-terminal --tab -- -r __ns:=/gcs_1 
            ]],
            shell=True
        ),
        ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 run swarm_load_carry gcs_user --ros-args -r __node:=gcs_user1 --params-file {config}"', #-r __ns:=/gcs_1
            ]],
            shell=True
        )
    ])