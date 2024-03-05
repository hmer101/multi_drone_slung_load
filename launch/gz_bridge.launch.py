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

import os, yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():   
    ## GET PARAMETERS
    config = os.path.join(
      get_package_share_directory('swarm_load_carry'),
      'config',
      'sim.yaml'
      )

    with open(config, 'r') as file:
        params = yaml.safe_load(file)
    
    topic_img_rgb = params["/**"]["ros__parameters"]["topic_img_rgb"]
    topic_cam_info = params["/**"]["ros__parameters"]["topic_cam_info_color"]
    
    first_drone_num = params["/**"]["ros__parameters"]["first_drone_num"]
    num_drones = params["/**"]["ros__parameters"]["num_drones"]
    num_cameras = params["/**"]["ros__parameters"]["num_cameras"]
    
    evaluate = params["/**"]["ros__parameters"]["evaluate"]
    load_pose_type = params["/**"]["ros__parameters"]["load_pose_type"]

    ## CONSTRUCT ARGUMENT LISTS
    gz_bridge_args = []
    im_bridge_args = []
    gz_bridge_remappings = []

    # Clock bridge (IGN -> ROS2)
    # Note clock bridge must be started to publish ground truth poses
    gz_bridge_args.append('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')
    
    
    for i in range(first_drone_num, num_drones+first_drone_num):
        # Cameras (IGN -> ROS2)
        if num_cameras > 0: #and load_pose_type == "visual": 
            # Camera images 
            im_bridge_args.append(f'/px4_{i}{topic_img_rgb}')

            # Camera info
            gz_bridge_args.append(f'/px4_{i}{topic_cam_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo')

        # Ground truth (IGN -> ROS2)
        if evaluate:
            gz_bridge_args.append(f'/model/swarm/model/x500_{i}/pose_static@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V')
            gz_bridge_remappings.append((f'/model/swarm/model/x500_{i}/pose_static', f'/px4_{i}/out/pose_ground_truth/gz'))
    
    if evaluate:
        # Remap load ground truth name
        gz_bridge_remappings.append(('/model/swarm/model/load/pose_static', '/load_1/out/pose_ground_truth/gz'))


    ## CONSTRUCT BRIDGES
    # Image bridge
    im_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=im_bridge_args,
        output='screen'
    )

    # Note only 1 gz_bridge can be started at a time it seems
    param_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=gz_bridge_args,
        remappings=gz_bridge_remappings,
        output='screen'
    )

    return LaunchDescription([
        im_bridge,
        param_bridge
    ])