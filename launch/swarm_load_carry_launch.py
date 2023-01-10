from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[[
                'gnome-terminal --tab -- bash -c "ros2 run swarm_load_carry drone --ros-args -r __ns:=/px4_1 -r __node:=droneX_newName"', #-r __node:=droneX_newName
            ]],
            shell=True
        ),
        ExecuteProcess(
            cmd=[[
                'gnome-terminal --tab -- bash -c "ros2 run swarm_load_carry gcs --ros-args -r __ns:=/gcs_1 -r __node:=gcs"',
            ]],
            shell=True
        ),
        # Node(
        #     package="swarm_load_carry",
        #     executable="drone",
        #     name="offboard_comp",
        #     namespace="px4_1",
        #     #output='screen',
        #     emulate_tty=True
        #     #prefix=["lxterminal -e"]
        # ),
        # Node(
        #     package="swarm_load_carry",
        #     executable="gcs",
        #     name="gcs",
        #     namespace="gcs_1",
        #     output='screen',
        #     emulate_tty=True
        #     #prefix=["lxterminal -e"]
        # )
    ])