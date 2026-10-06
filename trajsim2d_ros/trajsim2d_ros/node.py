"""ROS 2 node exposing the TrajSim2D simulator."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory

from trajsim2d_core.file_parser import load_canvas_from_file, save_canvas_to_file
from trajsim2d_core.environment import (
    generate_random_border,
    generate_random_convex_objects,
)
from trajsim2d_core.twodmanip import PlanarManipulator
from trajsim2d_core.geometry_canvas import GeometryCanvas
from trajsim2d_core.visualisation import initialise_visualisation
from trajsim2d_ros.action import Simulation


class TrajSim2DNode(Node):
    """Publish the simulator state and handle environment persistence."""

    VACANT = "VACANT"
    STARTUP = "STARTUP"
    IDLE = "IDLE"
    SIMULATING = "SIMULATING"
    GET_CURRENT_STATE = "GET_CURRENT_STATE"
    GENERATE_RANDOM_ENVIRONMENT = "GENERATE_RANDOM_ENVIRONMENT"
    LOAD_ENVIRONMENT_FROM_FILE = "LOAD_ENVIRONMENT_FROM_FILE"
    SET_TRAJECTORY_PROCESSING_SAVE_LOCATION = (
        "SET_TRAJECTORY_PROCESSING_SAVE_LOCATION"
    )
    SAVE_ENVIRONMENT = "SAVE_ENVIRONMENT"

    def __init__(self) -> None:
        super().__init__("trajsim2d")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("visualisation", True)

        self.canvas = None
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.visualisation_enabled = bool(self.get_parameter("visualisation").value)
        if self.resolution <= 0.0:
            raise ValueError("resolution must be positive")

        self.manipulator: Optional[PlanarManipulator] = None
        self.border: Optional[np.ndarray] = None
        self.obstacles: List[np.ndarray] = []
        self.base_transform = np.eye(3)
        self.positions: Optional[np.ndarray] = None
        self.velocities: Optional[np.ndarray] = None
        self.trajectory: Optional[List[Tuple[float, np.ndarray, np.ndarray]]] = None
        self.trajectory_start_time: Optional[float] = None
        self.save_location: Optional[Path] = None
        self.state = self.VACANT
        self._pending_goal: Optional[Simulation.Goal] = None
        self._pending_goal_error: Optional[str] = None
        self._goal_complete = threading.Event()
        self._callback_group = ReentrantCallbackGroup()
        self.visualisation_canvas: Optional[GeometryCanvas] = None
        self.visualisation_arm_ids: List[str] = []

        self.robot_state_pub = self.create_publisher(JointState, "robot_state", 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, "goal_pose", 10)
        self.cost_map_pub = self.create_publisher(OccupancyGrid, "cost_map", 1)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            "/motion_planner/trajectory",
            self._trajectory_callback,
            10,
        )
        self.action_server = ActionServer(
            self,
            Simulation,
            "simulation",
            execute_callback=self._execute_action,
            goal_callback=self._goal_callback,
            callback_group=self._callback_group,
        )
        rate = float(self.get_parameter("publish_rate").value)
        if rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        self.state_machine_timer = self.create_timer(
            0.05, self._state_machine_loop, callback_group=self._callback_group
        )
        self.interrupt_timer = self.create_timer(
            1.0 / rate, self._interrupt_loop, callback_group=self._callback_group
        )

    def _goal_callback(self, _goal_request: Simulation.Goal) -> GoalResponse:
        if self._pending_goal is not None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _trajectory_callback(self, message: JointTrajectory) -> None:
        if self.manipulator is None:
            self.get_logger().warning("Ignoring trajectory: no environment is loaded")
            return
        if self._pending_goal is not None:
            self.get_logger().warning("Ignoring trajectory while an action is running")
            return
        expected = [f"joint_{index + 1}" for index in range(self.manipulator.n)]
        names = list(message.joint_names)
        if names != expected:
            self.get_logger().error(
                "JointTrajectory joint_names must be joint_1, joint_2, etc. in order"
            )
            return
        if not message.points:
            self.get_logger().error("JointTrajectory must contain at least one point")
            return

        points: List[Tuple[float, np.ndarray, np.ndarray]] = []
        previous_time = -1.0
        for point in message.points:
            position = np.asarray(point.positions, dtype=float)
            velocity = np.asarray(point.velocities, dtype=float)
            if position.size != self.manipulator.n:
                self.get_logger().error(
                    "Each JointTrajectory point must contain every joint position"
                )
                return
            if velocity.size not in (0, self.manipulator.n):
                self.get_logger().error(
                    "JointTrajectory velocities must be empty or contain every joint"
                )
                return
            if velocity.size == 0:
                velocity = np.zeros(self.manipulator.n, dtype=float)
            point_time = float(point.time_from_start.sec) + (
                float(point.time_from_start.nanosec) * 1e-9
            )
            if point_time <= previous_time:
                self.get_logger().error(
                    "JointTrajectory point times must be strictly increasing"
                )
                return
            previous_time = point_time
            points.append((point_time, position, velocity))

        self.trajectory = points
        self.trajectory_start_time = time.monotonic()
        self.positions = points[0][1].copy()
        self.velocities = points[0][2].copy()
        self.state = self.SIMULATING

    def _load_environment(self, filename: Path) -> None:
        loaded = load_canvas_from_file(filename)
        self.manipulator = loaded.arm
        self.base_transform = np.asarray(loaded.base_transform, dtype=float)
        self.border = loaded.border
        self.obstacles = list(loaded.obstacles)
        self.start_config = loaded.start_config
        self.goal_config = loaded.end_config

        if self.visualisation_enabled:
            self._reset_visualisation()
            (
                self.canvas,
                self.base_transform,
                self.border_id,
                self.object_ids,
                self.arm_ids,
                self.start_config,
                self.goal_config,
            ) = initialise_visualisation(
                border=self.border,
                objs=self.obstacles,
                arm=self.manipulator,
                base_transform=self.base_transform,
                joint_config_1=self.start_config,
                joint_config_2=self.goal_config,
            )

        self._reset_robot_state()

    def _generate_random_environment(self) -> None:
        border_size = 5.0
        self.manipulator = PlanarManipulator()
        self.border = generate_random_border(border_size=border_size, smoothness=0.1)
        self.obstacles, _ = generate_random_convex_objects(
            border_size=border_size,
            border=self.border,
        )
        self.base_transform = np.eye(3)
        self.start_config = np.zeros(self.manipulator.n, dtype=float)
        self.goal_config = None
        if self.visualisation_enabled:
            self._reset_visualisation()
            (
                self.canvas,
                self.base_transform,
                self.border_id,
                self.object_ids,
                self.arm_ids,
                self.start_config,
                self.goal_config,
            ) = initialise_visualisation(
                border=self.border,
                objs=self.obstacles,
                arm=self.manipulator,
                base_transform=self.base_transform,
                joint_config_1=self.start_config,
                attempt_max=20,
            )

        self._reset_robot_state()

    def _reset_visualisation(self) -> None:
        if not self.visualisation_enabled:
            return
        if self.canvas is not None:
            self.canvas.close()

    def _reset_robot_state(self) -> None:
        self.positions = (
            np.asarray(self.start_config, dtype=float)
            if self.start_config is not None
            else np.zeros(self.manipulator.n, dtype=float)
        )
        self.velocities = np.zeros(self.manipulator.n, dtype=float)
        self.trajectory = None
        self.trajectory_start_time = None

    def _refresh_visualisation(self) -> None:
        if not self.visualisation_enabled:
            return
        if self.canvas is not None:
            self.canvas.refresh()
        return

    def _save_environment(self, filename: Path) -> None:
        if self.manipulator is None:
            raise RuntimeError("cannot save an environment before one is loaded")
        save_canvas_to_file(
            filename,
            self.manipulator,
            border=self.border,
            obstacles=self.obstacles,
            base_transform=self.base_transform,
            start_config=self.start_config,
            end_config=self.goal_config
        )

    def _execute_action(self, goal_handle: Simulation.Goal) -> Simulation.Result:
        request = goal_handle.request
        self._pending_goal = request
        self._pending_goal_error = None
        self._goal_complete.clear()
        while not self._goal_complete.wait(timeout=0.05):
            feedback = Simulation.Feedback()
            feedback.simulation_state = self.state
            goal_handle.publish_feedback(feedback)

        error = self._pending_goal_error
        self._pending_goal_error = None

        result = Simulation.Result()
        if error is not None:
            self.get_logger().error(error)
            goal_handle.abort()
            result.result = False
        else:
            goal_handle.succeed()
            result.result = True
        feedback = Simulation.Feedback()
        feedback.simulation_state = self.state
        goal_handle.publish_feedback(feedback)
        return result

    def _state_machine_loop(self) -> None:
        """Process one requested action and return to a passive state."""
        if self._pending_goal is None:
            # Visualise
            if self.state == self.SIMULATING:
                self._advance_trajectory()
            self._refresh_visualisation()
            return

        request = self._pending_goal
        self.state = self._goal_state(request.goal)
        try:
            if request.goal == Simulation.Goal.GET_CURRENT_STATE:
                pass
            elif request.goal == Simulation.Goal.GENERATE_RANDOM_ENVIRONMENT:
                self._generate_random_environment()
            elif request.goal == Simulation.Goal.LOAD_ENVIRONMENT_FROM_FILE:
                if not request.data:
                    raise ValueError("environment filename cannot be empty")
                self._load_environment(Path(request.data))
            elif request.goal == Simulation.Goal.SET_TRAJECTORY_PROCESSING_SAVE_LOCATION:
                if not request.data:
                    raise ValueError("save location cannot be empty")
                self.save_location = Path(request.data)
            elif request.goal == Simulation.Goal.SAVE_ENVIRONMENT:
                filename = (
                    Path(request.data) if request.data else self.save_location
                )
                if filename is None:
                    raise ValueError("no save location has been configured")
                self._save_environment(filename)
                self.save_location = filename
            else:
                raise ValueError(f"unknown action goal {request.goal}")
        except (OSError, ValueError, RuntimeError) as error:
            self._pending_goal_error = str(error)
        finally:
            self.state = self.IDLE if self.manipulator is not None else self.VACANT
            self._pending_goal = None
            self._goal_complete.set()

    def _goal_state(self, goal: int) -> str:
        states = {
            Simulation.Goal.GET_CURRENT_STATE: self.GET_CURRENT_STATE,
            Simulation.Goal.GENERATE_RANDOM_ENVIRONMENT: self.GENERATE_RANDOM_ENVIRONMENT,
            Simulation.Goal.LOAD_ENVIRONMENT_FROM_FILE: self.LOAD_ENVIRONMENT_FROM_FILE,
            Simulation.Goal.SET_TRAJECTORY_PROCESSING_SAVE_LOCATION: (
                self.SET_TRAJECTORY_PROCESSING_SAVE_LOCATION
            ),
            Simulation.Goal.SAVE_ENVIRONMENT: self.SAVE_ENVIRONMENT,
        }
        return states.get(goal, self.VACANT)

    def _interrupt_loop(self) -> None:
        """Advance simulation and publish ROS/visualisation outputs."""
        if self.manipulator is None or self.positions is None or self.velocities is None:
            return
        now = self.get_clock().now().to_msg()
        names = [f"joint_{index + 1}" for index in range(self.manipulator.n)]
        state = JointState()
        state.header.stamp = now
        state.header.frame_id = self.frame_id
        state.name = names
        state.position = self.positions.tolist()
        state.velocity = self.velocities.tolist()
        self.robot_state_pub.publish(state)

        tfs = self.manipulator.forward_kinematics(self.base_transform, self.positions)
        self.tf_pub.publish(TFMessage(transforms=self._transforms(tfs, now)))
        goal = PoseStamped()
        goal.header.stamp = now
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = float(tfs[-1][0, 2])
        goal.pose.position.y = float(tfs[-1][1, 2])
        goal.pose.orientation.z, goal.pose.orientation.w = self._yaw_quaternion(tfs[-1])
        self.goal_pose_pub.publish(goal)
        self.cost_map_pub.publish(self._cost_map(now))

    def _advance_trajectory(self) -> None:
        if self.trajectory is None or self.trajectory_start_time is None:
            self.state = self.IDLE
            return
        elapsed = time.monotonic() - self.trajectory_start_time
        if elapsed >= self.trajectory[-1][0]:
            _, self.positions, self.velocities = self.trajectory[-1]
            self.positions = self.positions.copy()
            self.velocities = self.velocities.copy()
            self.trajectory = None
            self.trajectory_start_time = None
            self.state = self.IDLE
            return

        for first, second in zip(self.trajectory, self.trajectory[1:]):
            if elapsed <= second[0]:
                duration = second[0] - first[0]
                fraction = (elapsed - first[0]) / duration
                self.positions = first[1] + fraction * (second[1] - first[1])
                self.velocities = first[2] + fraction * (second[2] - first[2])
                return

    def _transforms(self, tfs: Sequence[np.ndarray], stamp) -> List[TransformStamped]:
        transforms = []
        for index, tf in enumerate(tfs):
            message = TransformStamped()
            message.header.stamp = stamp
            message.header.frame_id = self.frame_id
            message.child_frame_id = self.base_frame if index == 0 else f"link_{index}"
            message.transform.translation.x = float(tf[0, 2])
            message.transform.translation.y = float(tf[1, 2])
            message.transform.rotation.z, message.transform.rotation.w = (
                self._yaw_quaternion(tf)
            )
            transforms.append(message)
        return transforms

    @staticmethod
    def _yaw_quaternion(tf: np.ndarray) -> Tuple[float, float]:
        yaw = math.atan2(float(tf[1, 0]), float(tf[0, 0]))
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def _cost_map(self, stamp) -> OccupancyGrid:
        scene_polygons = [polygon for polygon in [self.border, *self.obstacles] if polygon is not None]
        occupied_polygons = self.obstacles
        points = (
            np.vstack(scene_polygons)
            if scene_polygons
            else np.array([[0.0, 0.0], [1.0, 1.0]])
        )
        minimum = np.floor(points.min(axis=0) / self.resolution) * self.resolution
        maximum = np.ceil(points.max(axis=0) / self.resolution) * self.resolution
        width = max(1, int(round((maximum[0] - minimum[0]) / self.resolution)) + 1)
        height = max(1, int(round((maximum[1] - minimum[1]) / self.resolution)) + 1)
        data = np.full(width * height, -1, dtype=np.int8)
        for row in range(height):
            for column in range(width):
                point = minimum + self.resolution * np.array([column + 0.5, row + 0.5])
                if any(
                    self._inside_polygon(point, polygon)
                    for polygon in occupied_polygons
                ):
                    data[row * width + column] = 100
        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.info.resolution = self.resolution
        message.info.width = width
        message.info.height = height
        message.info.origin.position.x = float(minimum[0])
        message.info.origin.position.y = float(minimum[1])
        message.info.origin.orientation.w = 1.0
        message.data = data.tolist()
        return message

    @staticmethod
    def _inside_polygon(point: np.ndarray, polygon: np.ndarray) -> bool:
        x, y = point
        inside = False
        for first, second in zip(polygon, np.roll(polygon, -1, axis=0)):
            if (first[1] > y) != (second[1] > y):
                crossing = (second[0] - first[0]) * (y - first[1]) / (
                    second[1] - first[1]
                ) + first[0]
                if x < crossing:
                    inside = not inside
        return inside


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajSim2DNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
