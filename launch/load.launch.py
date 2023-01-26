import sys, os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess

DEFAULT_LOAD_ID=1
DEFAULT_ENV='sim'

def generate_launch_description():
    ## LAUNCH ARGUMENTS
    launch_arg_load_id = DeclareLaunchArgument(
      'load_id', default_value=str(DEFAULT_LOAD_ID)
    )
    launch_arg_sim_phys = DeclareLaunchArgument(
      'env', default_value=str(DEFAULT_ENV)
    )

    # Get arguments
    load_id = DEFAULT_LOAD_ID
    for arg in sys.argv:
        if arg.startswith("load_id:="):
            load_id = int(arg.split(":=")[1])

    env = DEFAULT_ENV
    for arg in sys.argv:
        if arg.startswith("env:="):
            env = int(arg.split(":=")[1])


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
        launch_arg_load_id,
        launch_arg_sim_phys,
        ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 run swarm_load_carry load --ros-args -r __node:=load{load_id} --params-file ~/px4_ros_com_ros2/src/swarm_load_carry/config/{env}.yaml"',  # -r __ns:=/load_{load_id}
            ]],
            shell=True
        )
    ])