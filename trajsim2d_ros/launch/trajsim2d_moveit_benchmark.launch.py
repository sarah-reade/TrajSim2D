from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    package_dir = get_package_share_directory("trajsim2d_ros")
    launch_file = Path(package_dir) / "launch" / "trajsim2d_moveit.launch.py"

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_file))
        ),
        Node(
            package="trajsim2d_ros",
            executable="auto_node",
            output="screen",
        ),
    ])