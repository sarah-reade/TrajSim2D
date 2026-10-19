from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [Node(package="trajsim2d_ros", executable="trajsim2d_moveit_node", output="screen"),
         Node(package="trajsim2d_ros", executable="moveit_launch_node", output="screen"),
         
         Node(package="tf2_ros",executable="static_transform_publisher",
            name="world_to_map_broadcaster",
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "world",
                "--child-frame-id", "map",
            ],output="screen")]
    )
