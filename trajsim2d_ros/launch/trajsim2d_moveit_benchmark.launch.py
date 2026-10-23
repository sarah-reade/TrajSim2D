from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    EmitEvent
)
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    package_dir = Path(get_package_share_directory("trajsim2d_ros"))

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_dir / "launch" / "trajsim2d_moveit.launch.py")
        )
    )

    auto_node = Node(
        package="trajsim2d_ros",
        executable="auto_node",
        output="screen",
    )

    shutdown_on_auto_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=auto_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason="auto_node exited"))
            ],
        )
    )

    return LaunchDescription([
        shutdown_on_auto_exit,
        moveit_launch,
        auto_node,
    ])