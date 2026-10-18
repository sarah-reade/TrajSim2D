"A ROS2 launch service, which launches moveit based on a folder name. "

import threading
from time import sleep
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
import yaml

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node as LaunchNode

from trajsim2d_ros.srv import RestartMoveit


# =========================
# 2. Request data structure
# =========================

@dataclass
class RestartRequest:
    folder: Path
    completed: threading.Event
    result: bool = False


# =========================
# 3. Server initialization
# =========================

class MoveitLaunchServer(Node):

    def __init__(self) -> None:
        super().__init__("moveit_launch_server")

        self._moveit_config_dir = None

        self._moveit_launch_service: Optional[LaunchService] = None

        self._pending_restart: Optional[RestartRequest] = None

        self._lock = threading.Lock()

        self._restart_service = self.create_service(
            RestartMoveit,
            "restart_moveit",
            self._handle_restart,
        )

        self.get_logger().info(
            "MoveIt launch server initialized"
        )

    # =========================
    # 4. Service callback
    # =========================

    def _handle_restart(self, request, response):
        _moveit_config_dir = Path(request.foldername).expanduser()

        self.get_logger().info(
            f"Received MoveIt restart request: {_moveit_config_dir}"
        )

        try:
            self._validate_config_dir(_moveit_config_dir)

        except Exception:
            self.get_logger().exception(
                f"Invalid MoveIt configuration directory: {_moveit_config_dir}"
            )
            response.result = False
            return response

        restart = RestartRequest(
            folder=_moveit_config_dir,
            completed=threading.Event(),
        )
        
        self._moveit_config_dir = _moveit_config_dir

        # Accept only one restart request at a time.
        with self._lock:
            
            print("-------------------- RESTARTING MOVEIT --------------------")
            
            if self._pending_restart is not None:
                self.get_logger().warning(
                    "A MoveIt restart is already pending"
                )
                response.result = False
                return response

            if self._moveit_launch_service is None:
                self.get_logger().error(
                    "MoveIt launch service is not running"
                )
                response.result = False
                return response

            self._pending_restart = restart

            # Ask the main thread's launch loop to return.
            self._moveit_launch_service.shutdown()

        # Wait until the main thread prepares the replacement launch.
        restart.completed.wait()

        
        response.result = restart.result

        return response

    # =========================
    # 5. Validate configuration
    # =========================

    @staticmethod
    def _validate_config_dir(config_dir: Path) -> None:
        required_files = (
            "robot.urdf",
            "robot.srdf",
            "kinematics.yaml",
            "ompl_planning.yaml",
        )

        if not config_dir.is_dir():
            raise FileNotFoundError(
                f"Configuration directory does not exist: {config_dir}"
            )

        for filename in required_files:
            path = config_dir / filename

            if not path.is_file():
                raise FileNotFoundError(
                    f"Required configuration file is missing: {path}"
                )

    # =========================
    # 6. Create MoveIt launch
    # =========================

    def _create_moveit_launch(
        self,
        config_dir: Path,
    ) -> LaunchService:

        self._validate_config_dir(config_dir)

        urdf = (
            config_dir / "robot.urdf"
        ).read_text(encoding="utf-8")

        srdf = (
            config_dir / "robot.srdf"
        ).read_text(encoding="utf-8")

        with (config_dir / "kinematics.yaml").open(
            encoding="utf-8"
        ) as f:
            kinematics = yaml.safe_load(f) or {}

        with (config_dir / "ompl_planning.yaml").open(
            encoding="utf-8"
        ) as f:
            ompl = yaml.safe_load(f) or {}

        parameters = {
            "robot_description": urdf,
            "robot_description_semantic": srdf,
            "robot_description_kinematics": kinematics,
            "planning_pipelines": ["ompl"],
            "default_planning_pipeline": "ompl",
            "ompl": ompl,
            "allow_trajectory_execution": False,
            "moveit_controller_manager": "moveit_fake_controller_manager/MoveItFakeControllerManager"
        }
        
        move_group = LaunchNode(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[parameters],
        )

        launch_service = LaunchService()

        launch_service.include_launch_description(
            LaunchDescription([move_group])
        )

        return launch_service

    # =========================
    # 7. Main-thread launch loop
    # =========================

    def run_moveit(self) -> None:
        """
        Run this function in the main thread.

        LaunchService.run() blocks until the launch is shut down.
        Restart requests are processed after the current launch exits.
        """

        request_to_complete = None

        while rclpy.ok():
            
            if self._moveit_config_dir is None:
                sleep(0.1)
                continue
            
            config_dir = self._moveit_config_dir 
            
            try:
                self.get_logger().info(
                    f"Launching MoveIt using: {config_dir}"
                )

                launch_service = self._create_moveit_launch(
                    config_dir
                )

            except Exception:
                self.get_logger().exception(
                    "Failed to configure MoveIt launch"
                )

                if request_to_complete is not None:
                    request_to_complete.result = False
                    request_to_complete.completed.set()
                    request_to_complete = None

                break

            with self._lock:
                self._moveit_launch_service = launch_service

            # The replacement launch has been constructed.
            # The service response indicates launch preparation,
            # not yet that MoveIt is fully ready.
            if request_to_complete is not None:
                request_to_complete.result = True
                request_to_complete.completed.set()
                request_to_complete = None

            try:
                # This runs in the main thread.
                return_code = launch_service.run()

                self.get_logger().info(
                    f"MoveIt launch exited with code {return_code}"
                )

            except Exception:
                self.get_logger().exception(
                    "MoveIt launch execution failed"
                )

            finally:
                with self._lock:
                    if self._moveit_launch_service is launch_service:
                        self._moveit_launch_service = None

            # Check whether shutdown was requested by the service.
            with self._lock:
                request_to_complete = self._pending_restart
                self._pending_restart = None

            if request_to_complete is None:
                # No restart request: the launch exited unexpectedly
                # or the application is shutting down.
                break

            # The next iteration constructs a new launch using
            # the requested folder.
            config_dir = request_to_complete.folder

            self.get_logger().info(
                f"Restarting MoveIt with: {config_dir}"
            )

    # =========================
    # 8. Cleanup
    # =========================

    def shutdown_moveit(self) -> None:
        with self._lock:
            launch_service = self._moveit_launch_service

            if launch_service is not None:
                launch_service.shutdown()

            if self._pending_restart is not None:
                self._pending_restart.result = False
                self._pending_restart.completed.set()
                self._pending_restart = None


# =========================
# 9. Main
# =========================

def main(args=None) -> None:
    rclpy.init(args=args)

    node = MoveitLaunchServer()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # ROS service callbacks run in this background thread.
    executor_thread = threading.Thread(
        target=executor.spin,
        name="moveit-service-executor",
        daemon=True,
    )
    executor_thread.start()

    try:
        # LaunchService.run() executes in the main thread.
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