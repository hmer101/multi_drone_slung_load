from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_load_carry',
            executable='logging_test',
            name='logging_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'DEBUG']
        )
    ])