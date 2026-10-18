"""ROS 2 node exposing the TrajSim2D simulator."""

from __future__ import annotations

import math
import threading
import tempfile
import time
from pathlib import Path
from typing import List, Optional, Sequence, Tuple
import numpy as np
import logging
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, TransformStamped
from shape_msgs.msg import Mesh, MeshTriangle
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, GoalResponse, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory

from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node as LaunchNode

from moveit_msgs.msg import PlanningScene, PlanningOptions, CollisionObject, MoveItErrorCodes
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.action import MoveGroup

from trajsim2d_core.file_parser import load_canvas_from_file, save_canvas_to_file, save_trajectory_to_file
from trajsim2d_core.environment import (
    generate_random_border,
    generate_random_convex_objects,
)
from trajsim2d_core.twodmanip import PlanarManipulator
from trajsim2d_core.geometry_canvas import GeometryCanvas
from trajsim2d_core.visualisation import initialise_visualisation, update_trajectory_visualisation
from trajsim2d_ros.action import Simulation, MoveitRequest
from trajsim2d_core.calculations import Trajectory, evaluate_trajectory_threaded,calculate_robot_state


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
    GENERATING_PLANNING_SCENE = "GENERATING_PLANNING_SCENE"
    SETTING_PLANNING_SCENE = "SETTING_PLANNING_SCENE"
    INITIALISING_MOVEIT_GOAL = "INITIALISING_MOVEIT_GOAL"
    CONVERTING_TRAJECTORY = "CONVERTING_TRAJECTORY"
    CANCELLING_GOAL = "CANCELLING_GOAL"    

    def __init__(self) -> None:
        super().__init__("trajsim2d")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("publish_rate", 30.0)

        self.canvas = None
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.resolution = float(self.get_parameter("resolution").value)
        if self.resolution <= 0.0:
            raise ValueError("resolution must be positive")

        self._eval_thread: Optional[threading.Thread] = None
        self.manipulator: Optional[PlanarManipulator] = None
        self.border: Optional[np.ndarray] = None
        self.obstacles: List[np.ndarray] = []
        self.base_transform = np.eye(3)
        self.positions: Optional[np.ndarray] = None
        self.velocities: Optional[np.ndarray] = None
        self.efforts: Optional[np.ndarray] = None
        self.start_config: Optional[np.ndarray] = None
        self.goal_config: Optional[np.ndarray] = None
        self.trajectory: Optional[List[Tuple[float, np.ndarray, np.ndarray]]] = None
        self.trajectory_start_time: Optional[float] = None
        self.save_location: Optional[Path] = Path("/tmp")
        self.state = self.VACANT
        self._pending_goal: Simulation.Goal | MoveitRequest.Goal | None = None
        self._pending_goal_error: Optional[str] = None
        self._goal_complete = threading.Event()
        self._callback_group_sim = ReentrantCallbackGroup()
        self._callback_group_mvit = ReentrantCallbackGroup()
        self.visualisation_canvas: Optional[GeometryCanvas] = None
        self.visualisation_arm_ids: List[str] = []
        self._cost_map_cache: Optional[OccupancyGrid] = None
        
        self._moveit_config_dir = Path("/tmp/moveit_config")
        self._moveit_thread: Optional[threading.Thread] = None
        self._moveit_launch_service = None
        self._moveit_stop_timeout = 10.0
        self._moveit_launch_error = None
        
        # MoveIt interfaces
        self._moveit_action_client = None
        self._planning_scene_client = None

        # Planning scene state
        self._planning_scene = None
        self._planning_scene_set = None
        self._planning_scene_request_pending = False
        self._planning_scene_future = None
        self._planning_scene_error = None
        self._moveit_scene_object_ids = set()

        # MoveIt action state
        self._moveit_goal_started = None
        self._moveit_goal_complete = None
        self._moveit_goal_feedback = None
        self._moveit_goal_error = None
        self._moveit_goal_handle = None
        self._moveit_goal_request_pending = False
        self._moveit_goal_future = None
        self._moveit_result_future = None
        self._moveit_result = None

        # Collision geometry settings
        self._moveit_planning_frame = "world"
        self._moveit_scene_z_min = -0.05
        self._moveit_scene_z_max = 0.05

        # Cancellation state
        self._moveit_cancel_future = None
        self._moveit_cancel_requested = False

        self.robot_state_pub = self.create_publisher(JointState, "robot_state", 10)
        self.goal_pose_pub = self.create_publisher(JointState, "goal_pose", 10)
        self.goal_ee_pose_pub = self.create_publisher(
            PoseStamped, "goal_ee_pose", 10
        )
        self.cost_map_pub = self.create_publisher(OccupancyGrid, "cost_map", 1)
        description_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.robot_description_pub = self.create_publisher(
            String, "robot_description", description_qos
        )
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.action_server_sim = ActionServer(
            self,
            Simulation,
            "simulation",
            execute_callback=self._execute_action_sim,
            goal_callback=self._goal_callback_sim,
            callback_group=self._callback_group_sim,
        )
        self.action_server_mvit = ActionServer(
            self,
            MoveitRequest,
            "moveit_request",
            execute_callback=self._execute_action_mvit,
            goal_callback=self._goal_callback_mvit,
            callback_group=self._callback_group_mvit,
        )
        rate = float(self.get_parameter("publish_rate").value)
        if rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        self.interrupt_timer_sim = self.create_timer(
            1.0 / rate, self._interrupt_loop_sim, callback_group=self._callback_group_sim
        )
        # self.interrupt_timer_mvit = self.create_timer(
        #     1.0 / rate, self._interrupt_loop_mvit, callback_group=self._callback_group_mvit
        # )

    def _goal_callback_sim(self, _goal_request: Simulation.Goal) -> GoalResponse:
        if self._pending_goal is not None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    def _goal_callback_mvit(self, _goal_request: MoveitRequest.Goal) -> GoalResponse:
            if self._pending_goal is not None:
                return GoalResponse.REJECT
            if self.state is self.VACANT:
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
            
        time = np.asarray([t for t, _, _ in points], dtype=float)
        q = np.asarray([position for _, position, _ in points], dtype=float)

        self.trajectory = Trajectory(
            time=time,
            q=q,
            base_tf=self.base_transform,
            attachment_end=1,
        )
        
        self.positions = points[0][1].copy()
        self.velocities = points[0][2].copy()
        
        self._eval_thread = evaluate_trajectory_threaded(self.trajectory, self.manipulator, self.obstacles)
        
        #self.state = self.SIMULATING

    def _load_environment(self, filename: Path) -> None:
        loaded = load_canvas_from_file(filename)
        self.manipulator = loaded.arm
        self.base_transform = np.asarray(loaded.base_transform, dtype=float)
        self.border = loaded.border
        self.obstacles = list(loaded.obstacles)
        self.start_config = loaded.start_config
        self.goal_config = loaded.end_config

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
        self._cost_map_cache = self._create_cost_map(
            self.get_clock().now().to_msg()
        )
        self._publish_robot_description()
        self._restart_moveit()

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
        self.goal_config = np.zeros(self.manipulator.n, dtype=float)
        
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
            attempt_max=20
        )

        self._reset_robot_state()
        self._cost_map_cache = self._create_cost_map(
            self.get_clock().now().to_msg()
        )
        self._publish_robot_description()
        self._restart_moveit()

    def _publish_robot_description(self) -> None:
        if self.manipulator is None:
            return
        temporary_path: Optional[Path] = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w+b", suffix=".urdf", delete=False
            ) as temporary_file:
                temporary_path = Path(temporary_file.name)
            self.manipulator.save_to_urdf(str(temporary_path))
            self.description = temporary_path.read_text(encoding="utf-8")
            message = String()
            message.data = self.description
            self.robot_description_pub.publish(message)
        finally:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)

    def _restart_moveit(self) -> None:
        self.get_logger().info("Restarting MoveIt")

        # 1. Stop the old launch completely.
        self._stop_moveit()

        try:
            # 2. Generate fresh configuration files.
            config_dir = self._generate_moveit_config()

            # 3. Launch MoveIt using the new configuration.
            self._start_moveit(config_dir)

        except Exception:
            self.get_logger().exception(
                "Failed to restart MoveIt"
            )
            raise
        
        self._initialise_moveit_interfaces()
        
    def _initialise_moveit_interfaces(self) -> None:
        if self._planning_scene_client is None:
            self._planning_scene_client = self.create_client(
                ApplyPlanningScene,
                "/apply_planning_scene",
            )

        if self._moveit_action_client is None:
            self._moveit_action_client = ActionClient(
                self,
                MoveGroup,
                "/move_action",
            )
        
    def _generate_moveit_config(self) -> Path:
        config_dir = Path("/tmp/moveit_config")

        # Validate the robot description.
        if not self.description:
            raise ValueError("self.description must contain URDF XML")

        urdf_root = ET.fromstring(self.description)

        if urdf_root.tag != "robot":
            raise ValueError("Robot description is not a valid URDF")

        # The SRDF must define planning groups for MoveIt.
        srdf = self.generate_srdf(self.description)

        if not srdf:
            raise ValueError(
                "self.semantic_description must contain SRDF XML "
                "with the robot's planning groups"
            )

        srdf_root = ET.fromstring(srdf)

        if srdf_root.tag != "robot":
            raise ValueError("Semantic description is not a valid SRDF")

        if srdf_root.attrib.get("name") != urdf_root.attrib.get("name"):
            raise ValueError("URDF and SRDF robot names do not match")

        config_dir.mkdir(parents=True, exist_ok=True)

        # Remove only files managed by this function.
        for filename in (
            "robot.urdf",
            "robot.srdf",
            "kinematics.yaml",
            "ompl_planning.yaml",
        ):
            (config_dir / filename).unlink(missing_ok=True)

        # Save the robot descriptions.
        (config_dir / "robot.urdf").write_text(
            self.description,
            encoding="utf-8",
        )

        (config_dir / "robot.srdf").write_text(
            srdf,
            encoding="utf-8",
        )

        # Generic kinematics configuration.
        # Add group-specific IK solvers here if required.
        with (config_dir / "kinematics.yaml").open("w") as f:
            yaml.safe_dump({}, f)

        # Generic OMPL configuration.
        ompl_config = {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/ResolveConstraintFrames "
                "default_planner_request_adapters/ValidateWorkspaceBounds "
                "default_planner_request_adapters/CheckStartStateBounds "
                "default_planner_request_adapters/CheckStartStateCollision"
            ),
            "start_state_max_bounds_error": 0.1,
            "planner_configs": {
                "RRTConnectkConfigDefault": {
                    "type": "geometric::RRTConnect",
                    "range": 0.0,
                },
            },
        }

        with (config_dir / "ompl_planning.yaml").open("w") as f:
            yaml.safe_dump(ompl_config, f, sort_keys=False)

        self.get_logger().info(
            f"Generated MoveIt configuration in {config_dir}"
        )

        return config_dir

    def generate_srdf(self,urdf: str) -> str:
        root = ET.fromstring(urdf)

        links = {
            link.attrib["name"]
            for link in root.findall("link")
        }

        joints = root.findall("joint")

        # Find the root link: a link that is never a joint's child.
        child_links = {
            joint.find("child").attrib["link"]
            for joint in joints
        }

        roots = links - child_links

        if len(roots) != 1:
            raise ValueError(f"Expected one root link, got {roots}")

        base_link = roots.pop()

        # Follow the serial chain from the base.
        current_link = base_link
        chain_joints = []

        while True:
            children = [
                joint for joint in joints
                if joint.find("parent").attrib["link"] == current_link
            ]

            if not children:
                break

            if len(children) != 1:
                raise ValueError(
                    f"Expected a serial chain at {current_link}"
                )

            joint = children[0]
            current_link = joint.find("child").attrib["link"]

            if joint.attrib["type"] in ("revolute", "continuous", "prismatic"):
                chain_joints.append(joint)

        if not chain_joints:
            raise ValueError("No movable joints found")

        tip_link = current_link
        robot_name = root.attrib["name"]

        srdf = ET.Element("robot", {"name": robot_name})

        group = ET.SubElement(srdf, "group", {"name": "manipulator"})
        ET.SubElement(
            group,
            "chain",
            {
                "base_link": base_link,
                "tip_link": tip_link,
            },
        )

        return ET.tostring(srdf, encoding="unicode")
        
    def _stop_moveit(self) -> None:
        service = self._moveit_launch_service
        thread = self._moveit_thread

        if service is None:
            if thread is not None and thread.is_alive():
                raise RuntimeError(
                    "MoveIt thread is alive but its LaunchService is missing"
                )
            self._moveit_thread = None
            return

        self.get_logger().info("Stopping MoveIt launch")

        # Request shutdown of the launch service and its launched actions.
        service.shutdown()

        if thread is not None:
            thread.join(timeout=self._moveit_stop_timeout)

            if thread.is_alive():
                raise RuntimeError(
                    "MoveIt launch did not stop within the timeout; "
                    "refusing to start another instance"
                )

        self._moveit_launch_service = None
        self._moveit_thread = None

        self.get_logger().info("MoveIt launch stopped")
    
    def _is_moveit_running(self) -> bool:
        return (
            self._moveit_thread is not None
            and self._moveit_thread.is_alive()
            and self._moveit_launch_error is None
        )
    
    def _start_moveit(self, config_dir: Path) -> None:
        if (
            self._moveit_thread is not None
            and self._moveit_thread.is_alive()
        ):
            raise RuntimeError("MoveIt launch is already running")

        urdf = (config_dir / "robot.urdf").read_text(
            encoding="utf-8"
        )
        srdf = (config_dir / "robot.srdf").read_text(
            encoding="utf-8"
        )

        with (config_dir / "kinematics.yaml").open() as f:
            kinematics = yaml.safe_load(f) or {}

        with (config_dir / "ompl_planning.yaml").open() as f:
            ompl = yaml.safe_load(f) or {}

        # Configure the OMPL pipeline and the MoveIt node.
        parameters = {
            "robot_description": urdf,
            "robot_description_semantic": srdf,
            "robot_description_kinematics": kinematics,
            "planning_pipelines": ["ompl"],
            "default_planning_pipeline": "ompl",
            "ompl": ompl,
        }

        move_group = LaunchNode(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[parameters],
        )

        service = LaunchService()
        service.include_launch_description(
            LaunchDescription([move_group])
        )

        self._moveit_launch_error = None

        def _run_launch():
            try:
                return_code = service.run()

                if return_code != 0:
                    self._moveit_launch_error = RuntimeError(
                        f"MoveIt launch exited with code {return_code}"
                    )

            except Exception as exc:
                self._moveit_launch_error = exc
                logging.exception("MoveIt launch failed")

        thread = threading.Thread(
            target=_run_launch,
            name="moveit-launch",
            daemon=True,
        )

        self._moveit_launch_service = service
        self._moveit_thread = thread

        thread.start()

        self.get_logger().info("MoveIt launch thread started")
    
    def _reset_visualisation(self) -> None:
        if self.canvas is not None:
            self.canvas.close()

    def _reset_robot_state(self) -> None:
        self.positions = (
            np.asarray(self.start_config, dtype=float)
            if self.start_config is not None
            else np.zeros(self.manipulator.n, dtype=float)
        )
        self.velocities = np.zeros(self.manipulator.n, dtype=float)
        self.efforts = np.zeros(self.manipulator.n, dtype=float)
        self.trajectory = None
        self.trajectory_start_time = None

    def _refresh_visualisation(self) -> None:
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

    def _execute_action_sim(self, goal_handle: Simulation.Goal) -> Simulation.Result:
        request = goal_handle.request
        self._pending_goal = request
        self._pending_goal_error = None
        self._goal_complete.clear()
        while not self._goal_complete.wait(timeout=0.05):
            feedback = Simulation.Feedback()
            feedback.node_state = self.state
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
        feedback.node_state = self.state
        goal_handle.publish_feedback(feedback)
        return result

    def _execute_action_mvit(self, goal_handle: MoveitRequest.Goal) -> MoveitRequest.Result:
        request = goal_handle
        self._pending_goal = request
        self._pending_goal_error = None
        self._goal_complete.clear()
        while not self._goal_complete.wait(timeout=0.05):
            feedback = MoveitRequest.Feedback()
            feedback.node_state = self.state
            goal_handle.publish_feedback(feedback)

        error = self._pending_goal_error
        self._pending_goal_error = None

        result = MoveitRequest.Result()
        if error is not None:
            self.get_logger().error(error)
            goal_handle.abort()
            result.result = False
        else:
            goal_handle.succeed()
            result.result = True
        feedback = MoveitRequest.Feedback()
        feedback.node_state = self.state
        goal_handle.publish_feedback(feedback)
        return result
    
    def _get_moveit_trajectory(self) -> None:
        
        while self._pending_goal is not None:
            if self._planning_scene is None:
                self.state = self.GENERATING_PLANNING_SCENE
                self._planning_scene = self._generate_planning_scene()
            elif self._planning_scene_set is None:
                self.state = self.SETTING_PLANNING_SCENE
                self._set_planning_scene(self._planning_scene)
            elif self._moveit_goal_started is None:
                self.state = self.INITIALISING_MOVEIT_GOAL
                self._start_moveit_goal(self._pending_goal)
            elif self._moveit_goal_complete is True:
                if self._moveit_result is not None:
                    self.state = self.CONVERTING_TRAJECTORY
                    self.trajectory = self._convert_trajectory(
                        self._moveit_result
                    )
                else:
                    self._pending_goal = None
                    if self._moveit_goal_error is not None:
                        self._pending_goal_error = (
                            "MOVEIT_ERROR_" + self._moveit_goal_error
                        )
                    else:
                        self._pending_goal_error = (
                            "UNKNOWN_ERROR_NO_TRAJECTORY"
                        )
            elif self.trajectory is None:
                self._pending_goal = None
                if self._moveit_goal_error is not None:
                    self._pending_goal_error = "MOVEIT_ERROR_" + self._moveit_goal_error
                else:
                    self._pending_goal_error = "UNKNOWN_ERROR_NO_TRAJECTORY"
            else:
                self._pending_goal = None
                self.state = self.SIMULATING
        
        
        if self._moveit_goal_started and not self._moveit_goal_complete:
            self._moveit_cancel_goal()
        self._planning_scene = None
        self._planning_scene_set = None
        self._moveit_goal_started = None
        self._moveit_goal_complete = None
        self._moveit_result = None
        self.trajectory = None
        
    def _generate_planning_scene(self) -> PlanningScene:
        scene_polygons = [
            polygon
            for polygon in [self.border, *self.obstacles]
            if polygon is not None
        ]

        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        current_ids = {
            "border" if i == 0 else f"obstacle_{i - 1}"
            for i, polygon in enumerate(scene_polygons)
        }

        # Remove objects that existed in the previous scene but no
        # longer exist in the current simulator environment.
        for object_id in self._moveit_scene_object_ids - current_ids:
            collision = CollisionObject()
            collision.id = object_id
            collision.header.frame_id = self._moveit_planning_frame
            collision.operation = CollisionObject.REMOVE

            planning_scene.world.collision_objects.append(collision)

        # Add or update current objects.
        for i, polygon in enumerate(scene_polygons):
            object_id = (
                "border" if i == 0 else f"obstacle_{i - 1}"
            )

            planning_scene.world.collision_objects.append(
                self._polygon_to_collision_object(
                    polygon,
                    object_id,
                )
            )

        self._moveit_scene_object_ids = current_ids

        return planning_scene
    
    def _polygon_to_collision_object(
        self,
        polygon: np.ndarray,
        object_id: str,
    ) -> CollisionObject:

        polygon, cap_triangles = self._triangulate_polygon(polygon)

        z_min = float(self._moveit_scene_z_min)
        z_max = float(self._moveit_scene_z_max)

        if z_max <= z_min:
            raise ValueError("Scene z_max must be greater than z_min")

        n = len(polygon)

        mesh = Mesh()

        # First N vertices are the bottom; next N are the top.
        for z in (z_min, z_max):
            for x, y in polygon:
                mesh.vertices.append(
                    Point(x=float(x), y=float(y), z=z)
                )

        # Bottom cap: reverse winding for outward-facing normals.
        for a, b, c in cap_triangles:
            triangle = MeshTriangle()
            triangle.vertex_indices = [c, b, a]
            mesh.triangles.append(triangle)

        # Top cap.
        for a, b, c in cap_triangles:
            triangle = MeshTriangle()
            triangle.vertex_indices = [a + n, b + n, c + n]
            mesh.triangles.append(triangle)

        # Side faces.
        for i in range(n):
            j = (i + 1) % n

            bottom_i = i
            bottom_j = j
            top_i = i + n
            top_j = j + n

            for indices in (
                (bottom_i, bottom_j, top_j),
                (bottom_i, top_j, top_i),
            ):
                triangle = MeshTriangle()
                triangle.vertex_indices = list(indices)
                mesh.triangles.append(triangle)

        collision = CollisionObject()
        collision.id = object_id
        collision.header.frame_id = self._moveit_planning_frame
        collision.operation = CollisionObject.ADD

        pose = Pose()
        pose.orientation.w = 1.0

        collision.meshes.append(mesh)
        collision.mesh_poses.append(pose)

        return collision
    
    def _triangulate_polygon(self, polygon: np.ndarray) -> list:
        polygon = np.asarray(polygon, dtype=float)

        if polygon.ndim != 2 or polygon.shape[1] != 2:
            raise ValueError("Polygon must have shape (N, 2)")

        if not np.all(np.isfinite(polygon)):
            raise ValueError("Polygon contains non-finite coordinates")

        # Remove a duplicated closing vertex.
        if (
            len(polygon) > 1
            and np.allclose(polygon[0], polygon[-1])
        ):
            polygon = polygon[:-1]

        if len(polygon) < 3:
            raise ValueError("A polygon needs at least 3 vertices")

        # Signed area: positive means counter-clockwise.
        area = 0.5 * np.sum(
            polygon[:, 0] * np.roll(polygon[:, 1], -1)
            - polygon[:, 1] * np.roll(polygon[:, 0], -1)
        )

        if abs(area) < 1e-12:
            raise ValueError("Polygon has zero area")

        # Ear clipping assumes counter-clockwise winding.
        if area < 0:
            polygon = polygon[::-1].copy()

        indices = list(range(len(polygon)))
        triangles = []

        def cross(a, b, c):
            return (
                (b[0] - a[0]) * (c[1] - a[1])
                - (b[1] - a[1]) * (c[0] - a[0])
            )

        def inside_triangle(p, a, b, c):
            eps = 1e-10
            return (
                cross(a, b, p) >= -eps
                and cross(b, c, p) >= -eps
                and cross(c, a, p) >= -eps
            )

        while len(indices) > 3:
            found_ear = False

            for i in range(len(indices)):
                prev_idx = indices[i - 1]
                curr_idx = indices[i]
                next_idx = indices[(i + 1) % len(indices)]

                a = polygon[prev_idx]
                b = polygon[curr_idx]
                c = polygon[next_idx]

                if cross(a, b, c) <= 1e-10:
                    continue

                if any(
                    inside_triangle(polygon[j], a, b, c)
                    for j in indices
                    if j not in (prev_idx, curr_idx, next_idx)
                ):
                    continue

                triangles.append((prev_idx, curr_idx, next_idx))
                indices.pop(i)
                found_ear = True
                break

            if not found_ear:
                raise ValueError(
                    "Could not triangulate polygon; check for "
                    "self-intersections or duplicate vertices"
                )

        triangles.append(tuple(indices))

        return polygon, triangles
    
    def _set_planning_scene(
        self,
        planning_scene: PlanningScene,
    ) -> None:
        self._initialise_moveit_interfaces()

        if self._planning_scene_request_pending:
            return

        if not self._planning_scene_client.service_is_ready():
            if not self._planning_scene_client.wait_for_service(
                timeout_sec=0.0
            ):
                # Retry on the next pass through the controller loop.
                return

        service_request = ApplyPlanningScene.Request()
        service_request.scene = planning_scene

        self._planning_scene_set = None
        self._planning_scene_request_pending = True
        self._planning_scene_error = None

        try:
            future = self._planning_scene_client.call_async(
                service_request
            )
            self._planning_scene_future = future
            future.add_done_callback(
                self._planning_scene_response_callback
            )

        except Exception as exc:
            self._planning_scene_request_pending = False
            self._planning_scene_error = str(exc)
            self._fail_moveit_request(
                f"PLANNING_SCENE_REQUEST_FAILED: {exc}"
            )
    
    def _planning_scene_response_callback(self, future) -> None:
        self._planning_scene_request_pending = False

        try:
            response = future.result()

            if response is not None and response.success:
                self._planning_scene_set = True
                self._planning_scene_error = None
                self.get_logger().info(
                    "MoveIt planning scene applied successfully"
                )
                return

            self._planning_scene_error = (
                "ApplyPlanningScene returned success=False"
            )

        except Exception as exc:
            self._planning_scene_error = str(exc)

        self._planning_scene_set = False

        self._fail_moveit_request(
            "PLANNING_SCENE_FAILED: "
            + str(self._planning_scene_error)
        )
    
    def _start_moveit_goal(self, request) -> None:
        self._initialise_moveit_interfaces()

        if self._moveit_goal_request_pending:
            return

        if self._moveit_goal_started is True:
            return

        if self._planning_scene_set is not True:
            return

        if not self._moveit_action_client.server_is_ready():
            self._moveit_goal_started = False
            self._moveit_goal_complete = True
            self._moveit_goal_error = "ACTION_SERVER_UNAVAILABLE"
            return

        self.save_location = request.foldername

        motion_plan_goal = MoveGroup.Goal()
        motion_plan_goal.request = request.request

        planning_options = PlanningOptions()
        planning_options.plan_only = True
        planning_options.look_around = False
        planning_options.replan = False

        # The scene has already been applied using ApplyPlanningScene.
        planning_options.planning_scene_diff.is_diff = True

        motion_plan_goal.planning_options = planning_options

        self._moveit_goal_request_pending = True
        self._moveit_goal_started = None
        self._moveit_goal_complete = None
        self._moveit_goal_feedback = None
        self._moveit_goal_error = None
        self._moveit_goal_handle = None
        self._moveit_result = None

        try:
            future = self._moveit_action_client.send_goal_async(
                motion_plan_goal,
                feedback_callback=self._moveit_feedback_callback,
            )

            self._moveit_goal_future = future
            future.add_done_callback(
                self._moveit_goal_response_callback
            )

        except Exception as exc:
            self._moveit_goal_request_pending = False
            self._moveit_goal_started = False
            self._moveit_goal_complete = True
            self._moveit_goal_error = f"GOAL_SEND_FAILED: {exc}"
    
    def _moveit_goal_response_callback(self, future) -> None:
        self._moveit_goal_request_pending = False

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._moveit_goal_started = False
            self._moveit_goal_complete = True
            self._moveit_goal_error = f"GOAL_RESPONSE_FAILED: {exc}"
            return

        if not goal_handle.accepted:
            self._moveit_goal_started = False
            self._moveit_goal_complete = True
            self._moveit_goal_error = "GOAL_REJECTED"
            self.get_logger().warning("MoveIt rejected the goal")
            return

        self._moveit_goal_handle = goal_handle
        self._moveit_goal_started = True

        self._moveit_result_future = goal_handle.get_result_async()
        self._moveit_result_future.add_done_callback(
            self._moveit_result_callback
        )

        self.get_logger().info("MoveIt goal accepted")
    
    def _moveit_feedback_callback(self, feedback_msg) -> None:
        self._moveit_goal_feedback = feedback_msg.feedback.state
    
    def _moveit_result_callback(self, future) -> None:
        try:
            wrapped_result = future.result()
            result = wrapped_result.result

            self._moveit_goal_complete = True

            if (
                result.error_code.val
                != MoveItErrorCodes.SUCCESS
            ):
                self._moveit_goal_error = (
                    f"PLANNING_FAILED_{result.error_code.val}"
                )
                self._moveit_result = None
                return

            self._moveit_result = result

        except Exception as exc:
            self._moveit_goal_complete = True
            self._moveit_goal_error = f"RESULT_FAILED: {exc}"
            self._moveit_result = None
    
    def _convert_trajectory(self,moveit_result:MoveGroup.Result) -> Trajectory:
        moveit_trajectory = moveit_result.planned_trajectory.joint_trajectory
        self._trajectory_callback(moveit_trajectory)
        return 
    
    def _moveit_cancel_goal(self) -> None:
        goal_handle = self._moveit_goal_handle

        if goal_handle is None or not goal_handle.accepted:
            return

        if self._moveit_goal_complete is True:
            return

        if self._moveit_cancel_requested:
            return

        self._moveit_cancel_requested = True

        try:
            self._moveit_cancel_future = (
                goal_handle.cancel_goal_async()
            )
            self._moveit_cancel_future.add_done_callback(
                self._moveit_cancel_response_callback
            )

        except Exception:
            self.get_logger().exception(
                "Failed to request MoveIt goal cancellation"
            )
    
    def _moveit_cancel_response_callback(self, future) -> None:
        try:
            response = future.result()

            if response.goals_canceling:
                self.get_logger().info(
                    "MoveIt goal cancellation accepted"
                )
            else:
                self.get_logger().warning(
                    "MoveIt did not accept the cancellation request"
                )

        except Exception:
            self.get_logger().exception(
                "MoveIt cancellation response failed"
            )
    
    def _fail_moveit_request(self, error: str) -> None:
        self._pending_goal_error = "MOVEIT_ERROR_" + error

        # False is a terminal sentinel in the controller loop below.
        self._pending_goal = False
    
    def _main_loop(self) -> None:
        """Process one requested action and return to a passive state."""
        if self._pending_goal is None:
            #print("BOOP: %s",self.state)
            # Visualise
            if self.state == self.SIMULATING:
                self._advance_trajectory()
            self._refresh_visualisation()
            return

        request = self._pending_goal
        try:
            if isinstance(request, MoveitRequest.Goal):
                self._get_moveit_trajectory()
            else:
                self.state = self._goal_state(request.goal)
            
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

    def _interrupt_loop_sim(self) -> None:
        """Advance simulation and publish ROS/visualisation outputs."""
        if (self.state is not self.IDLE and self.state is not self.SIMULATING) or self.manipulator is None or self.positions is None or self.velocities is None:
            return
        now = self.get_clock().now().to_msg()
        names = [f"joint_{index + 1}" for index in range(self.manipulator.n)]
        state = JointState()
        state.header.stamp = now
        state.header.frame_id = self.frame_id
        state.name = names
        state.position = self.positions.tolist()
        state.velocity = self.velocities.tolist()
        state.effort = self.efforts.tolist()
        self.robot_state_pub.publish(state)

        fk_tfs = self.manipulator.forward_kinematics(
            self.base_transform, self.positions
        )
        link_tfs = []
        for index in range(self.manipulator.n):
            link_tf = fk_tfs[index].copy()
            link_tf[:2, :2] = fk_tfs[index + 1][:2, :2]
            link_tfs.append(link_tf)
        tfs = [self.base_transform, *link_tfs]
        self.tf_pub.publish(TFMessage(transforms=self._transforms(tfs, now)))
        goal = JointState()
        goal.header.stamp = now
        goal.header.frame_id = self.frame_id
        goal.name = names
        goal.position = (
            self.goal_config.tolist()
            if self.goal_config is not None
            else self.positions.tolist()
        )
        self.goal_pose_pub.publish(goal)
        
        goal_config = (
            self.goal_config
            if self.goal_config is not None
            else self.positions
        )
        
        goal_ee_tf = self.manipulator.forward_kinematics(
            self.base_transform, goal_config
        )[-1]
        goal_ee_pose = PoseStamped()
        goal_ee_pose.header.stamp = now
        goal_ee_pose.header.frame_id = self.frame_id
        goal_ee_pose.pose.position.x = float(goal_ee_tf[0, 2])
        goal_ee_pose.pose.position.y = float(goal_ee_tf[1, 2])
        goal_ee_pose.pose.orientation.z, goal_ee_pose.pose.orientation.w = (
            self._yaw_quaternion(goal_ee_tf)
        )
        self.goal_ee_pose_pub.publish(goal_ee_pose)

        if self._cost_map_cache is not None:
            self.cost_map_pub.publish(self._cost_map_cache)

    def _advance_trajectory(self) -> None:
        
        if self.trajectory is None:
            self.state = self.IDLE
            return
        
        if self.trajectory_start_time is None:
            self.arm_ids = [None]
            self.trajectory_start_time = time.monotonic()
            
        
        elapsed = time.monotonic() - self.trajectory_start_time
        
        [self.arm_ids, done, index] = update_trajectory_visualisation(elapsed,self.canvas,self.manipulator,self.trajectory,self.arm_ids,border=self.border,objs=self.obstacles)
        
        if done:
            # check if calculation thread still running 
            if self._eval_thread.is_alive():
                pass
            else:
                # save trajectory
                foldername = self.save_location / time.strftime("%d_%m_%y_%H_%M_%S")
                
                save_trajectory_to_file(foldername=foldername,trajectory=self.trajectory,manip=self.manipulator)
                self._reset_robot_state()
                self.state = self.IDLE
        else:
            [self.positions,self.velocities,self.efforts] = calculate_robot_state(self.manipulator,self.trajectory,index)
            
        return

    def _transforms(self, tfs: Sequence[np.ndarray], stamp) -> List[TransformStamped]:
        transforms = []
        parent_tf = np.eye(3)
        for index, tf in enumerate(tfs):
            relative_tf = np.linalg.inv(parent_tf) @ tf
            message = TransformStamped()
            message.header.stamp = stamp
            message.header.frame_id = (
                self.frame_id
                if index == 0
                else self.base_frame
                if index == 1
                else f"link_{index - 1}"
            )
            message.child_frame_id = self.base_frame if index == 0 else f"link_{index}"
            message.transform.translation.x = float(relative_tf[0, 2])
            message.transform.translation.y = float(relative_tf[1, 2])
            message.transform.rotation.z, message.transform.rotation.w = (
                self._yaw_quaternion(relative_tf)
            )
            transforms.append(message)
            parent_tf = tf
        return transforms

    @staticmethod
    def _yaw_quaternion(tf: np.ndarray) -> Tuple[float, float]:
        yaw = math.atan2(float(tf[1, 0]), float(tf[0, 0]))
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def _create_cost_map(self, stamp) -> OccupancyGrid:
        scene_polygons = [
            polygon
            for polygon in [self.border, *self.obstacles]
            if polygon is not None
        ]

        points = (
            np.vstack(scene_polygons)
            if scene_polygons
            else np.array([[0.0, 0.0], [1.0, 1.0]])
        )

        resolution = self.resolution

        minimum = (
            np.floor(points.min(axis=0) / resolution) * resolution
        )
        maximum = (
            np.ceil(points.max(axis=0) / resolution) * resolution
        )

        width = max(
            1,
            int(round((maximum[0] - minimum[0]) / resolution)) + 1,
        )
        height = max(
            1,
            int(round((maximum[1] - minimum[1]) / resolution)) + 1,
        )

        # Cell centres.
        x = minimum[0] + resolution * (np.arange(width) + 0.5)
        y = minimum[1] + resolution * (np.arange(height) + 0.5)

        xx, yy = np.meshgrid(x, y)

        occupied = np.zeros((height, width), dtype=bool)

        # Obstacles.
        for polygon in self.obstacles:
            if polygon is None or len(polygon) < 3:
                continue

            occupied |= self._points_inside_polygon(
                xx,
                yy,
                polygon,
            )

        # Border.
        if self.border is not None and len(self.border) >= 2:
            occupied |= self._points_near_polygon_boundary(
                xx,
                yy,
                self.border,
                resolution,
            )

        data = np.full((height, width), -1, dtype=np.int8)
        data[occupied] = 100

        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id

        message.info.resolution = resolution
        message.info.width = width
        message.info.height = height

        message.info.origin.position.x = float(minimum[0])
        message.info.origin.position.y = float(minimum[1])
        message.info.origin.orientation.w = 1.0

        message.data = data.ravel().tolist()

        return message


    @staticmethod
    def _points_inside_polygon(
        xx: np.ndarray,
        yy: np.ndarray,
        polygon: np.ndarray,
    ) -> np.ndarray:
        """
        Vectorized ray-casting point-in-polygon test.
        """

        x = xx.ravel()
        y = yy.ravel()

        first = polygon
        second = np.roll(polygon, -1, axis=0)

        x1 = first[:, 0]
        y1 = first[:, 1]
        x2 = second[:, 0]
        y2 = second[:, 1]

        inside = np.zeros(x.shape, dtype=bool)

        for i in range(len(polygon)):
            yi = y1[i]
            yj = y2[i]
            xi = x1[i]
            xj = x2[i]

            crosses = (yi > y) != (yj > y)

            if not np.any(crosses):
                continue

            crossing_x = (
                (xj - xi) * (y[crosses] - yi) / (yj - yi) + xi
            )

            inside[crosses] ^= x[crosses] < crossing_x

        return inside.reshape(xx.shape)


    @staticmethod
    def _points_near_polygon_boundary(
        xx: np.ndarray,
        yy: np.ndarray,
        polygon: np.ndarray,
        resolution: float,
    ) -> np.ndarray:
        """
        Vectorized test for grid-cell centres near a polygon boundary.
        """

        x = xx.ravel()
        y = yy.ravel()

        near = np.zeros(x.shape, dtype=bool)

        threshold_squared = (resolution * np.sqrt(2.0) * 0.5) ** 2

        first = polygon
        second = np.roll(polygon, -1, axis=0)

        for p1, p2 in zip(first, second):
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]

            length_squared = dx * dx + dy * dy

            if length_squared == 0.0:
                distance_squared = (
                    (x - p1[0]) ** 2 +
                    (y - p1[1]) ** 2
                )
            else:
                projection = (
                    (x - p1[0]) * dx +
                    (y - p1[1]) * dy
                ) / length_squared

                projection = np.clip(projection, 0.0, 1.0)

                closest_x = p1[0] + projection * dx
                closest_y = p1[1] + projection * dy

                distance_squared = (
                    (x - closest_x) ** 2 +
                    (y - closest_y) ** 2
                )

            near |= distance_squared <= threshold_squared

            # Avoid doing more work once every cell is classified.
            if near.all():
                break

        return near.reshape(xx.shape)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajSim2DNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        while rclpy.ok():
            node._main_loop()
            time.sleep(0.01)
    finally:
        executor.shutdown()
        spin_thread.join()
        node.destroy_node()
        rclpy.shutdown()
