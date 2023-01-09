from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="swarm_load_carry",
            executable="drone",
            name="offboard_comp",
            namespace="px4_1"
        ),
        Node(
            package="swarm_load_carry",
            executable="gcs",
            name="gcs",
            namespace="gcs_1"
        )
    ])