# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Allows images published in gazebo to be forward to ROS2 messages

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():   
    gz_bridge_clock = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swarm_load_carry'), 'launch'),
         '/gz_bridge_clock.launch.py'])
      )
    
    # Gz - ROS Bridge for publishing ground truth poses
    bridge_gt = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (IGN -> ROS2)
            #'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # Load 1 (IGN -> ROS2)
            '/model/swarm/model/load/pose_static@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            #'/model/swarm/model/load/pose_static@geometry_msgs/msg/Pose[gz.msgs.Pose',

            # Drones (IGN -> ROS2)
            '/model/swarm/model/x500_1/pose_static@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            '/model/swarm/model/x500_2/pose_static@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            '/model/swarm/model/x500_3/pose_static@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            
        ],
        remappings=[
            ('/model/swarm/model/load/pose_static', '/load_1/out/pose_ground_truth/gz'),
            ('/model/swarm/model/x500_1/pose_static', '/x500_1/out/pose_ground_truth/gz'),
            ('/model/swarm/model/x500_2/pose_static', '/x500_2/out/pose_ground_truth/gz'),
            ('/model/swarm/model/x500_3/pose_static', '/x500_3/out/pose_ground_truth/gz'),
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_bridge_clock,
        bridge_gt
    ])