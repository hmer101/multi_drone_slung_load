import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ## INCLUDE LAUNCH FILES
    load = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/load.launch.py']),
      launch_arguments={'env': 'phys'}.items()
      )
    gcs = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/gcs.launch.py']),
      launch_arguments={'env': 'phys'}.items()
      )

    ## RUN LAUNCH FILES
    return LaunchDescription([
        load,
        gcs
    ])
