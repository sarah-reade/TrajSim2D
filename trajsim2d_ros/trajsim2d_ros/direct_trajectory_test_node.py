"""One-shot direct trajectory publisher for testing the simulator node."""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class DirectTrajectoryTestNode(Node):
    """Publish one smooth, direct trajectory between the observed states."""

    def __init__(self) -> None:
        super().__init__("direct_trajectory_test")
        self.declare_parameter("duration", 3.0)
        self.declare_parameter("samples", 21)

        duration = float(self.get_parameter("duration").value)
        samples = int(self.get_parameter("samples").value)
        if duration <= 0.0:
            raise ValueError("duration must be positive")
        if samples < 2:
            raise ValueError("samples must be at least 2")

        self._duration = duration
        self._samples = samples
        self._robot_state: Optional[JointState] = None
        self._goal_state: Optional[JointState] = None
        self._published = False

        self._trajectory_pub = self.create_publisher(
            JointTrajectory, "/motion_planner/trajectory", 10
        )
        self.create_subscription(
            JointState, "/robot_state", self._robot_state_callback, 10
        )
        self.create_subscription(
            JointState, "/goal_pose", self._goal_state_callback, 10
        )

        self.get_logger().info(
            "Waiting for /robot_state and /goal_pose to publish one direct trajectory"
        )

    def _robot_state_callback(self, message: JointState) -> None:
        if not self._published:
            self._robot_state = message
            self._try_publish()

    def _goal_state_callback(self, message: JointState) -> None:
        if not self._published:
            self._goal_state = message
            self._try_publish()

    def _try_publish(self) -> None:
        if self._robot_state is None or self._goal_state is None:
            return

        start = np.asarray(self._robot_state.position, dtype=float)
        goal = np.asarray(self._goal_state.position, dtype=float)
        if start.size == 0 or goal.size == 0:
            self.get_logger().error("Cannot create a trajectory from an empty state")
            return
        if start.size != goal.size:
            self.get_logger().error(
                "Robot and goal states contain different numbers of joints"
            )
            return

        names = list(self._robot_state.name)
        if len(names) != start.size:
            self.get_logger().error(
                "Robot state joint names and positions have different sizes"
            )
            return
        if list(self._goal_state.name) != names:
            self.get_logger().error(
                "Robot and goal states must contain the same joints in the same order"
            )
            return

        trajectory = JointTrajectory()
        trajectory.joint_names = names
        for index in range(self._samples):
            normalized_time = index / (self._samples - 1)
            smooth_time = normalized_time**2 * (3.0 - 2.0 * normalized_time)
            smooth_velocity = (
                6.0
                * normalized_time
                * (1.0 - normalized_time)
                / self._duration
            )

            point = JointTrajectoryPoint()
            point.positions = (
                start + smooth_time * (goal - start)
            ).tolist()
            point.velocities = (
                smooth_velocity * (goal - start)
            ).tolist()
            point.time_from_start = self._duration_message(
                normalized_time * self._duration
            )
            trajectory.points.append(point)

        self._trajectory_pub.publish(trajectory)
        self._published = True
        self.get_logger().info(
            f"Published one direct trajectory with {self._samples} points "
            f"over {self._duration:.2f} seconds"
        )
        
        raise("Created Trajectory, now Kicking")

    @staticmethod
    def _duration_message(seconds: float) -> Duration:
        whole_seconds = int(seconds)
        nanoseconds = int(round((seconds - whole_seconds) * 1e9))
        if nanoseconds == 1_000_000_000:
            whole_seconds += 1
            nanoseconds = 0
        message = Duration()
        message.sec = whole_seconds
        message.nanosec = nanoseconds
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DirectTrajectoryTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
