import os, yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

ENV='sim' #phys

def generate_launch_description():
    ## INCLUDE LAUNCH FILES
    drones = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/drones.launch.py'])
      )
    load = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/load.launch.py']),
      launch_arguments={'env': ENV}.items()
      )
    gcs = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/gcs.launch.py']),
         launch_arguments={'env': ENV}.items()
      )

    ## RUN LAUNCH FILES
    return LaunchDescription([
        drones,
        load,
        gcs    
    ])