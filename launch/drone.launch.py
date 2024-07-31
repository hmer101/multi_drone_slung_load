import sys, os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

DEFAULT_DRONE_ID=1
DEFAULT_ENV='phys'

def generate_launch_description():
    ## LAUNCH ARGUMENTS
    launch_arg_drone_id = DeclareLaunchArgument(
      'drone_id', default_value=str(DEFAULT_DRONE_ID)
    )
    launch_arg_sim_phys = DeclareLaunchArgument(
      'env', default_value=str(DEFAULT_ENV)
    )

    # Get arguments  
    drone_id = LaunchConfiguration('drone_id')

    env = DEFAULT_ENV
    for arg in sys.argv:
        if arg.startswith("env:="):
            env = arg.split(":=")[1]


    ## GET PARAMETERS
    config = None

    if env=="sim":
      config = os.path.join(
        get_package_share_directory('multi_drone_slung_load'),
        'config',
        'sim.yaml'
        )
    elif env=="phys":
       config = os.path.join(
        get_package_share_directory('multi_drone_slung_load'),
        'config',
        'phys.yaml'
        ) 

    ## LAUNCH
    return LaunchDescription([
        launch_arg_drone_id,
        launch_arg_sim_phys,
        Node(
         package='multi_drone_slung_load',
         executable='drone',
         namespace=PythonExpression(["'/px4_' + str(", drone_id, ")"]),
         name=PythonExpression(["'drone' + str(", drone_id, ")"]),
         output='screen',
         parameters=[config]
        )
    ])
