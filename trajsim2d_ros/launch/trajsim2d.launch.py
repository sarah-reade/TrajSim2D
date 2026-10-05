from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [Node(package="trajsim2d_ros", executable="trajsim2d_node", output="screen")]
    )
