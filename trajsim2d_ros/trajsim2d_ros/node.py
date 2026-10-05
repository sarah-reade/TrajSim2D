"""ROS 2 node exposing the TrajSim2D simulator."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

try:
    from trajectory_msgs.msg import JointVelocity
except ImportError:  # The fallback makes the wrapper usable without an external custom message.
    from trajsim2d_ros.msg import JointVelocity

from trajsim2d_core.file_parser import load_canvas_from_file, save_canvas_to_file
from trajsim2d_core.twodmanip import PlanarManipulator
from trajsim2d_ros.action import Simulation


class TrajSim2DNode(Node):
    """Publish the simulator state and handle environment persistence."""

    VACANT = "VACANT"
    STARTUP = "STARTUP"
    IDLE = "IDLE"
    SIMULATING = "SIMULATING"

    def __init__(self) -> None:
        super().__init__("trajsim2d")
        self.declare_parameter("urdf", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("publish_rate", 30.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.resolution = float(self.get_parameter("resolution").value)
        if self.resolution <= 0.0:
            raise ValueError("resolution must be positive")

        self.manipulator: Optional[PlanarManipulator] = None
        self.border: Optional[np.ndarray] = None
        self.obstacles: List[np.ndarray] = []
        self.base_transform = np.eye(3)
        self.positions: Optional[np.ndarray] = None
        self.velocities: Optional[np.ndarray] = None
        self.save_location: Optional[Path] = None
        self.state = self.VACANT

        self.robot_state_pub = self.create_publisher(JointState, "robot_state", 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, "goal_pose", 10)
        self.cost_map_pub = self.create_publisher(OccupancyGrid, "cost_map", 1)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.velocity_sub = self.create_subscription(
            JointVelocity, "/motion_planner/trajectory", self._velocity_callback, 10
        )
        self.action_server = ActionServer(
            self,
            Simulation,
            "simulation",
            execute_callback=self._execute_action,
            goal_callback=self._goal_callback,
        )
        rate = float(self.get_parameter("publish_rate").value)
        if rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        self.timer = self.create_timer(1.0 / rate, self._publish)

        urdf = str(self.get_parameter("urdf").value)
        if urdf:
            self._load_urdf(Path(urdf))

    def _goal_callback(self, _goal_request: Simulation.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _velocity_callback(self, message: JointVelocity) -> None:
        if self.manipulator is None:
            self.get_logger().warning("Ignoring trajectory: no environment is loaded")
            return
        names = list(getattr(message, "name", getattr(message, "joint_names", [])))
        values = np.asarray(getattr(message, "velocity", []), dtype=float)
        expected = [f"joint_{index + 1}" for index in range(self.manipulator.n)]
        if values.size != len(names) or values.size != self.manipulator.n:
            self.get_logger().error(
                "JointVelocity must contain one velocity for every manipulator joint"
            )
            return
        if names and names != expected:
            by_name = dict(zip(names, values))
            if set(by_name) != set(expected):
                self.get_logger().error("JointVelocity contains unknown or missing joint names")
                return
            values = np.asarray([by_name[name] for name in expected], dtype=float)
        self.velocities = values
        self.state = self.SIMULATING if np.any(np.abs(values) > 1e-12) else self.IDLE

    def _load_urdf(self, filename: Path) -> None:
        self.manipulator = PlanarManipulator(filename=str(filename))
        self.base_transform = np.asarray(self.manipulator.base_tf, dtype=float)
        self.positions = np.zeros(self.manipulator.n, dtype=float)
        self.velocities = np.zeros(self.manipulator.n, dtype=float)
        self.border = None
        self.obstacles = []
        self.state = self.IDLE

    def _load_environment(self, filename: Path) -> None:
        self.state = self.STARTUP
        canvas = load_canvas_from_file(filename)
        self.manipulator = canvas.arm
        self.base_transform = np.asarray(canvas.base_transform, dtype=float)
        self.border = canvas.border
        self.obstacles = list(canvas.obstacles)
        self.positions = (
            np.asarray(canvas.start_config, dtype=float)
            if canvas.start_config is not None
            else np.zeros(self.manipulator.n, dtype=float)
        )
        self.velocities = np.zeros(self.manipulator.n, dtype=float)
        self.state = self.IDLE

    def _save_environment(self, filename: Path) -> None:
        if self.manipulator is None:
            raise RuntimeError("cannot save an environment before one is loaded")
        save_canvas_to_file(
            filename,
            self.manipulator,
            border=self.border,
            obstacles=self.obstacles,
            base_transform=self.base_transform,
            start_config=self.positions,
        )

    async def _execute_action(self, goal_handle: Simulation.Goal) -> Simulation.Result:
        result = Simulation.Result()
        try:
            if goal_handle.request.goal == Simulation.Goal.SET_SAVE_LOCATION:
                if not goal_handle.request.data:
                    raise ValueError("save location cannot be empty")
                self.save_location = Path(goal_handle.request.data)
            elif goal_handle.request.goal == Simulation.Goal.LOAD_ENVIRONMENT:
                if not goal_handle.request.data:
                    raise ValueError("environment filename cannot be empty")
                self._load_environment(Path(goal_handle.request.data))
            elif goal_handle.request.goal == Simulation.Goal.SAVE_ENVIRONMENT:
                filename = (
                    Path(goal_handle.request.data)
                    if goal_handle.request.data
                    else self.save_location
                )
                if filename is None:
                    raise ValueError("no save location has been configured")
                self._save_environment(filename)
                self.save_location = filename
            elif goal_handle.request.goal == Simulation.Goal.REQUEST_CURRENT_STATE:
                pass
            else:
                raise ValueError(f"unknown action goal {goal_handle.request.goal}")
            result.result = True
            goal_handle.succeed()
        except (OSError, ValueError, RuntimeError) as error:
            self.get_logger().error(str(error))
            if self.manipulator is None:
                self.state = self.VACANT
            else:
                self.state = self.IDLE
            result.result = False
            goal_handle.abort()

        feedback = Simulation.Feedback()
        feedback.simulation_state = self.state
        goal_handle.publish_feedback(feedback)
        result.result = bool(result.result)
        return result

    def _publish(self) -> None:
        if self.manipulator is None or self.positions is None or self.velocities is None:
            return
        dt = self.timer.timer_period_ns * 1e-9
        if self.state == self.SIMULATING:
            self.positions = self.positions + self.velocities * dt
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
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
