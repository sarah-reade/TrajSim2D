from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [Node(package="trajsim2d_ros", executable="trajsim2d_moveit_node", output="screen"),
         Node(package="trajsim2d_ros", executable="moveit_launch_node", output="screen")]
    )
