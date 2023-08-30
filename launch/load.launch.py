import sys, os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression

DEFAULT_LOAD_ID=1
DEFAULT_ENV='phys'

# Note "odd" launchfile as using some parameters before runtime
def generate_launch_description():
    ## LAUNCH ARGUMENTS
    launch_arg_load_id = DeclareLaunchArgument(
      'load_id', default_value=str(DEFAULT_LOAD_ID)
    )
    launch_arg_sim_phys = DeclareLaunchArgument(
      'env', default_value=str(DEFAULT_ENV)
    )

    # load_id_value = LaunchConfiguration('load_id')
    # sim_phys_value = LaunchConfiguration('env')

    #print(f'sim_phys_value: {sim_phys_value.perform(None)}')

    # Get arguments
    load_id = DEFAULT_LOAD_ID
    for arg in sys.argv:
        if arg.startswith("load_id:="):
            load_id = int(arg.split(":=")[1])

    env = DEFAULT_ENV
    for arg in sys.argv:
        if arg.startswith("env:="):
            env = str(arg.split(":=")[1])


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
                f'gnome-terminal --tab -- bash -c "ros2 run swarm_load_carry load --ros-args -r __node:=load{load_id} --params-file {config}"',  # -r __ns:=/load_{load_id}
            ]],
            shell=True
        )
        #ExecuteProcess(cmd=cmd, shell=True)
    ])



# config = os.path.join(
#       get_package_share_directory('swarm_load_carry'),
#       'config',
#       'sim.yaml'
#       )

#     with open(config, 'r') as file:
#         params = yaml.safe_load(file)
    
#     num_drones = params["/**"]["ros__parameters"]["num_drones"]
#     first_drone_num = params["/**"]["ros__parameters"]["first_drone_num"]

#     ## LAUNCH BODY
#     # Launch each drone in a new terminal by calling the single drone launch file multiple times in new terminals
#     launch_description = []

#     for i in range(first_drone_num, num_drones+first_drone_num):
#         launch_description.append(ExecuteProcess(
#             cmd=[[
#                 f'gnome-terminal --tab -- bash -c "ros2 launch swarm_load_carry load.launch.py load_id:={1} env:=sim"',
#             ]],
#             shell=True
#         ))