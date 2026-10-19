"""Run MoveIt on the first restart service call and restart it on later calls."""

import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node as LaunchNode
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from trajsim2d_ros.srv import RestartMoveit


@dataclass
class RestartRequest:
    folder: Path
    completed: threading.Event
    result: bool = False


class MoveitLaunchServer(Node):
    """Expose a service to start or replace the MoveIt launch."""

    def __init__(self) -> None:
        super().__init__("moveit_launch_server")
        self._moveit_launch_service: Optional[LaunchService] = None
        self._pending_restart: Optional[RestartRequest] = None
        self._restart_event = threading.Event()
        self._lock = threading.Lock()
        self._restart_service = self.create_service(
            RestartMoveit, "restart_moveit", self._handle_restart
        )
        self.get_logger().info(
            "MoveIt launch server initialized; MoveIt will start on the first service call"
        )

    def _handle_restart(self, request, response):
        config_dir = Path(request.foldername).expanduser().resolve()
        self.get_logger().info(f"Received MoveIt restart request: {config_dir}")

        try:
            self._validate_config_dir(config_dir)
        except Exception:
            self.get_logger().exception(
                f"Invalid MoveIt configuration directory: {config_dir}"
            )
            response.result = False
            return response

        restart = RestartRequest(folder=config_dir, completed=threading.Event())
        with self._lock:
            if self._pending_restart is not None:
                self.get_logger().warning("A MoveIt restart is already pending")
                response.result = False
                return response

            self._pending_restart = restart
            active_launch = self._moveit_launch_service
            self._restart_event.set()

        # On subsequent calls, interrupt the currently running LaunchService.
        # On the first call active_launch is None, so the main loop wakes and starts it.
        if active_launch is not None:
            active_launch.shutdown()

        # The response reports whether the replacement launch was constructed,
        # not whether move_group has finished initializing.
        while rclpy.ok() and not restart.completed.wait(timeout=0.1):
            pass
        response.result = restart.result
        return response

    @staticmethod
    def _validate_config_dir(config_dir: Path) -> None:
        required_files = (
            "robot.urdf",
            "robot.srdf",
            "kinematics.yaml",
            "ompl_planning.yaml",
        )
        if not config_dir.is_dir():
            raise FileNotFoundError(f"Configuration directory does not exist: {config_dir}")
        for filename in required_files:
            path = config_dir / filename
            if not path.is_file():
                raise FileNotFoundError(f"Required configuration file is missing: {path}")

    def _create_moveit_launch(self, config_dir: Path) -> LaunchService:
        self._validate_config_dir(config_dir)
        urdf = (config_dir / "robot.urdf").read_text(encoding="utf-8")
        srdf = (config_dir / "robot.srdf").read_text(encoding="utf-8")
        with (config_dir / "kinematics.yaml").open(encoding="utf-8") as f:
            kinematics = yaml.safe_load(f) or {}
        with (config_dir / "ompl_planning.yaml").open(encoding="utf-8") as f:
            ompl = yaml.safe_load(f) or {}

        parameters = {
            "robot_description": urdf,
            "robot_description_semantic": srdf,
            "robot_description_kinematics": kinematics,
            "planning_pipelines": ["ompl"],
            "default_planning_pipeline": "ompl",
            "ompl": ompl,
            "allow_trajectory_execution": False,
            "moveit_controller_manager": (
                "moveit_fake_controller_manager/MoveItFakeControllerManager"
            ),
        }
        move_group = LaunchNode(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[parameters],
        )
        launch_service = LaunchService()
        launch_service.include_launch_description(LaunchDescription([move_group]))
        return launch_service

    def run_moveit(self) -> None:
        """Run in the main thread; remain idle until the first service request."""
        while rclpy.ok():
            self._restart_event.wait(timeout=0.1)
            if not rclpy.ok():
                break

            with self._lock:
                restart = self._pending_restart
                if restart is None:
                    self._restart_event.clear()
                    continue
                self._pending_restart = None
                self._restart_event.clear()

            launch_service = None
            try:
                self.get_logger().info(f"Launching MoveIt using: {restart.folder}")
                launch_service = self._create_moveit_launch(restart.folder)
                with self._lock:
                    self._moveit_launch_service = launch_service
                    # A newer request may have arrived while the launch was built.
                    pending_arrived = self._pending_restart is not None
                restart.result = True
                restart.completed.set()

                if pending_arrived:
                    launch_service.shutdown()
                return_code = launch_service.run()
                self.get_logger().info(f"MoveIt launch exited with code {return_code}")
            except Exception:
                self.get_logger().exception("Failed to configure or run MoveIt launch")
                restart.result = False
                restart.completed.set()
            finally:
                with self._lock:
                    if self._moveit_launch_service is launch_service:
                        self._moveit_launch_service = None

            # If a service request caused the launch to stop, the next iteration
            # consumes that queued request and starts the replacement.
            with self._lock:
                has_pending = self._pending_restart is not None
                if not has_pending:
                    self._restart_event.clear()
            if not has_pending:
                # An unexpected MoveIt exit should leave the service available;
                # it will launch again only when another request arrives.
                continue

    def shutdown_moveit(self) -> None:
        with self._lock:
            launch_service = self._moveit_launch_service
            pending = self._pending_restart
            self._pending_restart = None
            self._restart_event.set()

        if launch_service is not None:
            launch_service.shutdown()
        if pending is not None:
            pending.result = False
            pending.completed.set()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveitLaunchServer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin, name="moveit-service-executor", daemon=True
    )
    executor_thread.start()
    try:
        node.run_moveit()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_moveit()
        executor.shutdown()
        executor_thread.join()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()