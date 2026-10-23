#!/usr/bin/env python3

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient

from moveit_msgs.msg import MotionPlanRequest
from trajsim2d_ros.action import MoveitRequest, Simulation


class MoveitClient(Node):

    ENVIRONMENTS = ["/home/ewalker/Downloads/moveit_safe_env/world.canvas"]
    PLANNERS = {
        "ompl": ["SBLkConfigDefault","ESTkConfigDefault","RRTConnectkConfigDefault"]
    }

    def __init__(self):
        super().__init__("moveit_client")
        self.declare_parameter("results_folder", "/home/ewalker/Downloads/moveit/results/")
        self.declare_parameter("num_planning_attempts", 1)
        self.declare_parameter("planning_time", 10.0)
        self.declare_parameter("goal_tolerance", 0.01)
        self.declare_parameter("group_name", "manipulator")

        self.results_folder = self.get_parameter("results_folder").value
        self.num_planning_attempts = self.get_parameter("num_planning_attempts").value
        self.planning_time = self.get_parameter("planning_time").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.group_name = self.get_parameter("group_name").value

        self._moveit_action_client = ActionClient(
            self,
            MoveitRequest,
            "/moveit_request",  # Replace with your action name
        )
        
        self._sim_action_client = ActionClient(
            self,
            Simulation,
            "/simulation",  # Replace with your action name
        )
        
        self.valid_simulation_goals = {
            value
            for name, value in vars(Simulation.Goal).items()
            if name.isupper() and isinstance(value, int)
        }
        

    def send_simulation_goal(self, goal:int,data:str=""):
        # Wait for the action server
        self.get_logger().info("Waiting for Simulation action server...")
        
        if not self._sim_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server unavailable")
            return None

        request = Simulation.Goal()
        if goal not in self.valid_simulation_goals:
            self.get_logger().error(f"Simulation goal invalid: {goal}")
            return None
        
        request.goal = goal
        request.data = data
        
        
        self.get_logger().info("Sending simulation goal...")
        
        send_future = self._sim_action_client.send_goal_async(request,
            feedback_callback=self.feedback_callback,
        )
        
        result = self.wait_for_goal_complete(send_future)
        
        if result is not None:
            self.get_logger().info(f"Planning result: {result.result}")
        
        return result
        
    def send_moveit_goal(self, request: MotionPlanRequest, foldername: str):
        # Wait for the action server
        self.get_logger().info("Waiting for MoveIt action server...")

        if not self._moveit_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server unavailable")
            return None

        # Populate the action goal
        goal = MoveitRequest.Goal()
        goal.request = request
        goal.foldername = foldername

        self.get_logger().info("Sending planning request...")

        # Send goal and wait for acceptance/rejection
        send_future = self._moveit_action_client.send_goal_async(goal,
            feedback_callback=self.feedback_callback,
        )

        result = self.wait_for_goal_complete(send_future)
        
        if result is not None:
            self.get_logger().info(f"Planning result: {result.result}")
        
        return result
    
    def feedback_callback(self,feedback_msg):
        self.get_logger().info(f"Action Feedback (Node State): {feedback_msg.feedback.node_state}")
        self.sim_node_state = feedback_msg.feedback.node_state
    
    def wait_for_goal_complete(self,send_future:Future) -> bool:
        
        rclpy.spin_until_future_complete(self, send_future)
        
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            return None

        self.get_logger().info("Goal accepted; waiting for result...")

        # Wait for the action result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        response = result_future.result()

        if response is None:
            self.get_logger().error("No result received")
            return None

        result = response.result
        
        return result
    
    def get_sim_node_state(self) -> str:
        self.send_simulation_goal(Simulation.Goal.GET_CURRENT_STATE)
        return self.sim_node_state
                        
    def build_motion_plan_request(self,pipeline:str,planner:str) -> MotionPlanRequest:
        request = MotionPlanRequest()
        request.group_name = self.group_name
        request.pipeline_id = pipeline
        request.planner_id = planner
        request.num_planning_attempts = self.num_planning_attempts
        request.allowed_planning_time = self.planning_time
        return request
                
    def generate_folder(self, environment: str, planner_id: str) -> str:
        env_path = Path(environment).expanduser()
        results_path = Path(self.results_folder).expanduser()

        environment_name = f"{env_path.parent.name}_{env_path.stem}"

        return str(
            results_path / environment_name / planner_id
        )
    
    def main(self):
        # start simulation
        for env in self.ENVIRONMENTS:
            skip = False
            
            if self.send_simulation_goal(Simulation.Goal.LOAD_ENVIRONMENT_FROM_FILE,env):
                for pipeline, planners in self.PLANNERS.items():
                    if skip: 
                        break
                    
                    for planner in planners:
                        if skip: 
                            break
                        
                        try:
                            if self.get_sim_node_state() == "VACANT":
                                self.get_logger().error(f"Simulation Vacant After Loading: {env}")
                                skip = True
                                continue
                                
                            while self.get_sim_node_state() != "IDLE":
                                self.get_logger().info(f"Waiting for simulation to become idle. Simulation node state: {self.sim_node_state}")
                                
                            request = self.build_motion_plan_request(pipeline,planner)
                            foldername = self.generate_folder(env,planner)
                            self.send_moveit_goal(request,foldername)
                        except Exception as ex:
                            self.get_logger().error(
                                f"Failed to process environment '{env}' "
                                f"with pipeline '{pipeline}' and planner '{planner}': {ex}"
                            )
                        
        
        while self.get_sim_node_state() != "IDLE":
            self.get_logger().info(f"Waiting for simulation to become idle. Simulation node state: {self.sim_node_state}")
                        

            
            


def main(args=None):
    rclpy.init(args=args)

    client = MoveitClient()

    try:
        client.main()
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()