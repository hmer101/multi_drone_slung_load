from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution

import sys

DEFAULT_DRONE_NUM=1
DEFAULT_FIRST_DRONE_NUM=1

def generate_launch_description():
    ## LAUNCH ARGUMENTS
    launch_arg_num_drones = DeclareLaunchArgument(
      'num_drones', default_value=str(DEFAULT_DRONE_NUM) #TextSubstitution(text='1')
    )
    launch_arg_first_drone = DeclareLaunchArgument(
      'first_drone_num', default_value=str(DEFAULT_FIRST_DRONE_NUM)
    )

    #launch_config_num_drones = LaunchConfiguration('num_drones') # Currently can't get int value from this so using workaround below 
                                                    # (means need to launch this launchfile using ExecuteProccess in the higher-level launch file)
    # Set number of drones and drone start number
    num_drones = DEFAULT_DRONE_NUM
    first_drone_num = DEFAULT_FIRST_DRONE_NUM
    for arg in sys.argv:
        if arg.startswith("num_drones:="):
            num_drones = int(arg.split(":=")[1])
        elif arg.startswith("first_drone_num:="):
            first_drone_num = int(arg.split(":=")[1])


    ## LAUNCH BODY
    # Launch each drone in a new terminal
    launch_description = []
    launch_description.append(launch_arg_num_drones)
    launch_description.append(launch_arg_first_drone)

    for i in range(first_drone_num, num_drones+first_drone_num):
        launch_description.append(ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 run swarm_load_carry drone --ros-args -r __ns:=/px4_{i} -r __node:=drone{i}"',
            ]],
            shell=True
        ))
    
    return LaunchDescription(launch_description)