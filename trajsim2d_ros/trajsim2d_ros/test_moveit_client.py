#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.msg import MotionPlanRequest
from trajsim2d_ros.action import MoveitRequest


class MoveitClient(Node):

    def __init__(self):
        super().__init__("moveit_client")

        self._action_client = ActionClient(
            self,
            MoveitRequest,
            "/moveit_request",  # Replace with your action name
        )

    def send_goal(self, request: MotionPlanRequest, foldername: str):
        # Wait for the action server
        self.get_logger().info("Waiting for MoveIt action server...")

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server unavailable")
            return

        # Populate the action goal
        goal = MoveitRequest.Goal()
        goal.request = request
        goal.foldername = foldername

        self.get_logger().info("Sending planning request...")

        # Send goal and wait for acceptance/rejection
        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            return

        self.get_logger().info("Goal accepted; waiting for result...")

        # Wait for the action result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        response = result_future.result()

        if response is None:
            self.get_logger().error("No result received")
            return

        result = response.result

        self.get_logger().info(f"Planning result: {result.result}")


def main(args=None):
    rclpy.init(args=args)

    client = MoveitClient()

    # Build the request
    request = MotionPlanRequest()
    request.group_name = "manipulator"
    request.pipeline_id = "ompl"
    request.planner_id = "BKPIECEkConfigDefault"
    request.num_planning_attempts = 5
    request.allowed_planning_time = 5.0

    # The server generates the actual goal constraints in your implementation.
    foldername = "/tmp/moveit_plans"

    try:
        client.send_goal(request, foldername)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()