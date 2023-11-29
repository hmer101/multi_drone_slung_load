import os, yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch.substitutions import LaunchConfiguration

ENV='sim'
TERM_CMD= '' #'gnome-terminal --tab --'


def generate_launch_description():
    ## INCLUDE LAUNCH FILES
    drones = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/drones.launch.py'])
      )
    
    image_bridge = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/image_bridge.launch.py'])
      )
    
    gz_bridge = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/gz_bridge.launch.py'])
      )
    
    slung_pose_estimation_visual = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('slung_pose_estimation'), 'launch'),
         '/estimator.launch.py']),
         launch_arguments={'env': '{ENV}'}.items()
      )

    load = ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 launch swarm_load_carry load.launch.py load_id:={1} env:={ENV}"',
            ]],
            shell=True
        )
    
    gcs = ExecuteProcess(
            cmd=[[
                f'gnome-terminal --tab -- bash -c "ros2 launch swarm_load_carry gcs.launch.py env:={ENV}"',
            ]],
            shell=True
        )

    ## RUN LAUNCH FILES
    return LaunchDescription([
        image_bridge,
        gz_bridge,
        slung_pose_estimation_visual,
        drones,
        load,
        gcs    
    ])
