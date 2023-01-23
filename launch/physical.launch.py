import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource



## USE ENVIRONMENT VARS TO DEFINE DRONE NUM ON EACH RPI: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#environment-variables


# NUM_DRONES_TEMP=3
# FIRST_DRONE_NUM_TEMP=1

# def generate_launch_description():
#     config = os.path.join(
#       get_package_share_directory('swarm_load_carry'),
#       'config',
#       'sim.yaml'
#       )

#     return LaunchDescription([
#         # ExecuteProcess(
#         #     cmd=[[
#         #         f'gnome-terminal --tab -- bash -c "ros2 launch swarm_load_carry drones.launch.py num_drones:={NUM_DRONES_TEMP} first_drone_num:={FIRST_DRONE_NUM_TEMP}"',
#         #     ]],
#         #     shell=True
#         # ),
#         Node(
#          package='swarm_load_carry',
#          executable='gcs',
#          namespace='gcs_1',
#          name='gcs',
#          parameters=[config]
#       )
#     ])