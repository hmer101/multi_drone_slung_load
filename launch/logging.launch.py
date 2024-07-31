import sys, os, yaml

from ament_index_python.packages import get_package_share_directory
import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument

DEFAULT_ENV='phys'


def generate_launch_description():
    ## LAUNCH ARGUMENTS
    launch_arg_sim_phys = DeclareLaunchArgument(
      'env', default_value=str(DEFAULT_ENV)
    )
    
    # Get arguments  
    env = DEFAULT_ENV
    for arg in sys.argv:
        if arg.startswith("env:="):
            env = arg.split(":=")[1]

    ## GET PARAMETERS
    config = None
    filename_rosbag = None

    if env=="sim":
        config = os.path.join(
            get_package_share_directory('multi_drone_slung_load'),
            'config',
            'sim.yaml'
            )
      
        filename_rosbag = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    elif env=="phys":
        config = os.path.join(
            get_package_share_directory('multi_drone_slung_load'),
            'config',
            'phys.yaml'
            ) 
        
        filename_rosbag = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S") #'rosbag'

    with open(config, 'r') as file:
        params = yaml.safe_load(file)
    
    dir_rosbag = params["/**"]["ros__parameters"]["dir_rosbag"]

    # Generate rosbag path - use current time for filename if simulation, or defined name if physical
    dir_rosbag = os.path.join(dir_rosbag, filename_rosbag)
    
    return LaunchDescription([
        launch_arg_sim_phys,
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/tf', '/tf_static', '--output', f'{dir_rosbag}'],
            output='screen'
        ),
    ])
