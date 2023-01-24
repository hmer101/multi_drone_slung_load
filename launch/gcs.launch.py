import os, sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess

DEFAULT_ENV='sim'

def generate_launch_description():
    ## LAUNCH ARGUMENTS
    launch_arg_sim_phys = DeclareLaunchArgument(
      'env', default_value=str(DEFAULT_ENV)
    )

    # Get arguments
    env = DEFAULT_ENV
    for arg in sys.argv:
        if arg.startswith("env:="):
            env = int(arg.split(":=")[1])


    ## GET PARAMETERS
    # config = None

    # if env=="sim":
    #   config = os.path.join(
    #     get_package_share_directory('swarm_load_carry'),
    #     'config',
    #     'sim.yaml'
    #     )
    # elif env=="phys":
    #    config = os.path.join(
    #     get_package_share_directory('swarm_load_carry'),
    #     'config',
    #     'phys.yaml'
    #     ) 

    ## LAUNCH
    return LaunchDescription([
        # Node(
        #  package='swarm_load_carry',
        #  executable='gcs',
        #  namespace='gcs_1',
        #  name='gcs',
        #  output='screen',
        #  #emulate_tty=True,
        #  parameters=[config]
        # )

        ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 run swarm_load_carry gcs --ros-args --params-file ~/px4_ros_com_ros2/src/swarm_load_carry/config/{env}.yaml"',
            ]],
            shell=True
        )
    ])