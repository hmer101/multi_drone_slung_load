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
    
    # Gazebo
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_version': '7',
            'gz_args': '-r default.sdf'
        }.items(),
    )

    # RQt
    rqt = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=[LaunchConfiguration('image_topic')],
        condition=IfCondition(LaunchConfiguration('rqt'))
    )

    # Bridges
    # Fill out the arguments for the image_bridge and cam_info_bridge nodes
    im_bridge_args = []
    cam_info_bridge_args = []
    cam_info_bridge_remap = []

    for i in range(first_drone_num, num_drones+first_drone_num):
        im_bridge_args.append(f'/px4_{i}{topic_img_rgb}')
        cam_info_bridge_args.append(f'/px4_{i}/out/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo')
        #cam_info_bridge_remap.append((f'/px4_{i}/camera_info',f'/px4_{i}/{topic_cam_info}'))
  
    
    im_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=im_bridge_args, #[f'/px4_1{topic_img_rgb}', f'/px4_2{topic_img_rgb}', f'/px4_3{topic_img_rgb}'], #'/x500_1/camera_info', '/x500_2/camera_info', '/x500_3/camera_info'
        output='screen'
    )

    # Note clock bridge must be started to publish camera info: '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
    # cam_info_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=cam_info_bridge_args,
    #     #remappings=cam_info_bridge_remap,
    #     output='screen'
    # )

    cam_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (IGN -> ROS2)
            #'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            
            # Cameras (IGN -> ROS2)
            '/px4_1/out/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/px4_2/out/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/px4_3/out/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    return LaunchDescription([
        #gz_sim,
        DeclareLaunchArgument('rqt', default_value='true',
                              description='Open RQt.'),
        DeclareLaunchArgument('image_topic', default_value=f'/px4_1{topic_img_rgb}',
                              description='Topic to start viewing in RQt.'),
        im_bridge,
        cam_info_bridge,
        rqt
    ])
