import sys, os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

DEFAULT_DRONE_ID=1

def generate_launch_description():
    ## LAUNCH ARGUMENTS
    launch_arg_drone_id = DeclareLaunchArgument(
      'drone_id', default_value=str(DEFAULT_DRONE_ID)
    )

    # Get arguments  
    drone_id = LaunchConfiguration('drone_id')

    ## GET PARAMETERS
    config = os.path.join(
        get_package_share_directory('swarm_load_carry'),
        'config',
        'phys.yaml'
        ) 

    ## LAUNCH
    return LaunchDescription([
        launch_arg_drone_id,
        Node(
         package='realsense2_camera',
         executable='realsense2_camera_node',
         namespace=PythonExpression(["'/px4_' + str(", drone_id, ")"]),
         name=PythonExpression(["'camera' + str(", drone_id, ")"]),
         output='screen',
         parameters=[config]
        )
    ])
