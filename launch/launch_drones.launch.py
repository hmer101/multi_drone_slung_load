from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution

import sys

DEFAULT_DRONE_NUM=1
START_DRONE_NUM=1

def generate_launch_description():
    ## LAUNCH ARGUMENTS
    launch_arg_num_drones = DeclareLaunchArgument(
      'num_drones', default_value=str(DEFAULT_DRONE_NUM) #TextSubstitution(text='1')
    )
    #launch_config_num_drones = LaunchConfiguration('num_drones') # Currently can't get int value from this so using workaround below 
                                                    # (means need to launch this launchfile using ExecuteProccess in the higher-level launch file)
    # Set number of drones
    num_drones = DEFAULT_DRONE_NUM
    for arg in sys.argv:
        if arg.startswith("num_drones:="):
            num_drones = int(arg.split(":=")[1])


    ## LAUNCH BODY
    # Launch each drone in a new terminal
    launch_description = []
    launch_description.append(launch_arg_num_drones)

    for i in range(START_DRONE_NUM, num_drones+START_DRONE_NUM):
        launch_description.append(ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 run swarm_load_carry drone --ros-args -r __ns:=/px4_{i} -r __node:=drone{i}"',
            ]],
            shell=True
        ))
    
    return LaunchDescription(launch_description)