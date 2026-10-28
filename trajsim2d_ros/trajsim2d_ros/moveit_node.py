"""ROS 2 node exposing the TrajSim2D simulator."""

from __future__ import annotations

from copy import deepcopy
import math
import re
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
from collections.abc import Mapping

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, TransformStamped
from shape_msgs.msg import SolidPrimitive
from shape_msgs.msg import Mesh, MeshTriangle
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, GoalResponse, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String, ColorRGBA
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory
from rosidl_runtime_py.convert import message_to_ordereddict


from moveit_msgs.msg import PlanningScene, PlanningSceneWorld,PlanningOptions, CollisionObject, MoveItErrorCodes, MotionPlanRequest, Constraints, TrajectoryConstraints, JointConstraint, RobotState, ObjectColor, DisplayRobotState
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
from trajsim2d_core.calculations import Trajectory, evaluate_trajectory_threaded,calculate_robot_state
from trajsim2d_core.shape_utils import decompose_to_convex_shapes
from trajsim2d_ros.action import Simulation, MoveitRequest
from trajsim2d_ros.srv import RestartMoveit


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
    
    COLORS = {
        "red":         ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
        "green":       ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
        "blue":        ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
        "yellow":      ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),
        "orange":      ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0),
        "purple":      ColorRGBA(r=0.5, g=0.0, b=0.5, a=1.0),
        "cyan":        ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),
        "magenta":     ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0),
        "white":       ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
        "black":       ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),
        "grey":        ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0),
        "pink":        ColorRGBA(r=1.0, g=0.4, b=0.7, a=1.0),
    }
    
    PLUGIN_LIST = ["ompl","pilz_industrial_motion_planner","chomp","stomp","trajopt"]
    PLANNER_CONFIGS = {
        "ompl_interface/OMPLPlanner": {
            "SBLkConfigDefault": {
                "type": "geometric::SBL",
                "range": 0.0,
            },
            "ESTkConfigDefault": {
                "type": "geometric::EST",
                "range": 0.0,
            },
            "LBKPIECEkConfigDefault": {
                "type": "geometric::LBKPIECE",
                "range": 0.0,
            },
            "BKPIECEkConfigDefault": {
                "type": "geometric::BKPIECE",
                "range": 0.0,
            },
            "KPIECEkConfigDefault": {
                "type": "geometric::KPIECE",
                "range": 0.0,
            },
            "RRTkConfigDefault": {
                "type": "geometric::RRT",
                "range": 0.0,
            },
            "RRTConnectkConfigDefault": {
                "type": "geometric::RRTConnect",
                "range": 0.0,
            },
            "RRTstarkConfigDefault": {
                "type": "geometric::RRTstar",
                "range": 0.0,
            },
            "TRRTkConfigDefault": {
                "type": "geometric::TRRT",
                "range": 0.0,
            },
            "PRMkConfigDefault": {
                "type": "geometric::PRM",
                "range": 0.0,
            },
            "PRMstarkConfigDefault": {
                "type": "geometric::PRMstar",
                "range": 0.0,
            },
            "FMTkConfigDefault": {
                "type": "geometric::FMT",
                "range": 0.0,
            },
            "BFMTkConfigDefault": {
                "type": "geometric::BFMT",
                "range": 0.0,
            },
            "PDSTkConfigDefault": {
                "type": "geometric::PDST",
                "range": 0.0,
            },
            "STRIDEkConfigDefault": {
                "type": "geometric::STRIDE",
                "range": 0.0,
            },
            "BiTRRTkConfigDefault": {
                "type": "geometric::BiTRRT",
                "range": 0.0,
            },
            "LBTRRTkConfigDefault": {
                "type": "geometric::LBTRRT",
                "range": 0.0,
            },
            "BiESTkConfigDefault": {
                "type": "geometric::BiEST",
                "range": 0.0,
            },
            "ProjESTkConfigDefault": {
                "type": "geometric::ProjEST",
                "range": 0.0,
            },
            "LazyPRMkConfigDefault": {
                "type": "geometric::LazyPRM",
                "range": 0.0,
            },
            "SPARSkConfigDefault": {
                "type": "geometric::SPARS",
                "range": 0.0,
            },
            "SPARStwokConfigDefault": {
                "type": "geometric::SPARStwo",
                "range": 0.0,
            },
            "AnytimePathShortening": {
                "type": "geometric::AnytimePathShortening",
                "range": 0.0,
            },
        },
        # Pilz Industrial Motion Planner
        "pilz_industrial_motion_planner": {
            "PTP": {
                "type": "pilz_industrial_motion_planner/CommandPlanner",
            },
            "LIN": {
                "type": "pilz_industrial_motion_planner/CommandPlanner",
            },
            "CIRC": {
                "type": "pilz_industrial_motion_planner/CommandPlanner",
            },
        },
        # CHOMP
        "chomp_interface/CHOMPPlanner": {
            "chomp": {
                "type": "chomp_interface/CHOMPPlanner",
            },
        },

        # STOMP
        "stomp_moveit/StompPlanner": {
            "stomp": {
                "type": "stomp_moveit/StompPlanner",
            },
        },

        # TrajOpt (only if a compatible plugin is installed)
        "trajopt_interface/TrajOptPlanner": {
            "trajopt": {
                "type": "trajopt_interface/TrajOptPlanner",
            },
        }
    }

    def __init__(self) -> None:
        super().__init__("trajsim2d")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("goal_tolerance", 0.01)
        self.declare_parameter("robot_description", "")
        self.declare_parameter("display_trajectory", False)

        self.canvas = None
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.resolution = float(self.get_parameter("resolution").value)
        if self.resolution <= 0.0:
            raise ValueError("resolution must be positive")
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        if self.goal_tolerance <= 0.0:
            raise ValueError("goal tolerance must be positive")
        self.display_trajectory = bool(self.get_parameter("display_trajectory").value)

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
        self.trajectory: Optional[Trajectory] = None
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
        self.pipeline_id: str = None
        self.config_dir: Path = None
        
        # MoveIt interfaces
        self._moveit_action_client = None
        self._planning_scene_client = None

        # Planning scene state
        self._planning_scene = None
        self._planning_scene_set = None
        self._planning_scene_request_pending = False
        self._planning_scene_future = None
        self._moveit_error = None
        self._moveit_scene_object_ids = set()

        # MoveIt action state
        self._moveit_goal_started = None
        self._moveit_goal_complete = None
        self._moveit_goal_feedback = None
        self._moveit_error = None
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
        
        self._relaunch_moveit_client = self.create_client(
            RestartMoveit,
            "restart_moveit",
        )

        self.robot_state_pub = self.create_publisher(JointState, "robot_state", 10)
        self.display_robot_state_pub = self.create_publisher(DisplayRobotState, "display_robot_state", 10)
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
        self.semantic_robot_description_pub = self.create_publisher(
            String, "robot_description_semantic", description_qos
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

    def _goal_callback_sim(self, _goal_request: Simulation.Goal) -> GoalResponse:
        if _goal_request.goal == Simulation.Goal.GET_CURRENT_STATE:
            return GoalResponse.ACCEPT     
        if self._pending_goal is not None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    def _goal_callback_mvit(self, _goal_request: MoveitRequest.Goal) -> GoalResponse:
            if self._pending_goal is not None:
                return GoalResponse.REJECT
            if self.state is self.VACANT:
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT
        
    def _joint_trajectory_info(self, jt: JointTrajectory) -> str:
        """Return a readable summary of a JointTrajectory."""
        lines = [
            "JointTrajectory:",
            f"  Joint names: {list(jt.joint_names)}",
            f"  Number of points: {len(jt.points)}",
        ]

        for i, point in enumerate(jt.points):
            duration = (
                point.time_from_start.sec
                + point.time_from_start.nanosec * 1e-9
            )

            lines.append(f"  Point {i}:")
            lines.append(f"    Time from start: {duration:.3f} s")
            lines.append(f"    Positions: {list(point.positions)}")
            lines.append(f"    Velocities: {list(point.velocities)}")
            lines.append(f"    Accelerations: {list(point.accelerations)}")
            lines.append(f"    Effort: {list(point.effort)}")

        return "\n".join(lines)

    def _trajectory_callback(self, message: JointTrajectory) -> None:
        self.get_logger().debug(self._joint_trajectory_info(message))
        
        if self.manipulator is None:
            self.get_logger().warning("Ignoring trajectory: no environment is loaded")
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
            position = np.asarray(point.positions, dtype=float)*-1
            velocity = np.asarray(point.velocities, dtype=float)*-1
            if position.size != self.manipulator.n:
                self.get_logger().error(
                    "Each JointTrajectory point must contain every joint position. Joint Trajectory:" + self._joint_trajectory_info(message)
                )
                return
            if velocity.size not in (0, self.manipulator.n):
                self.get_logger().error(
                    "JointTrajectory velocities must be empty or contain every joint. Joint Trajectory:" + self._joint_trajectory_info(message)
                )
                return
            if velocity.size == 0:
                velocity = np.zeros(self.manipulator.n, dtype=float)
            point_time = float(point.time_from_start.sec) + (
                float(point.time_from_start.nanosec) * 1e-9
            )
            if point_time <= previous_time:
                self.get_logger().error(
                    "JointTrajectory point times must be strictly increasing. Joint Trajectory:" + self._joint_trajectory_info(message)
                )
                return
            previous_time = point_time
            points.append((point_time, position, velocity))
            
        time = np.asarray([t for t, _, _ in points], dtype=float)
        q = np.asarray([position for _, position, _ in points], dtype=float)

        try:
            self.trajectory = Trajectory(
                time=time,
                q=q,
                base_tf=self.base_transform,
                attachment_end=1,
            )
        except Exception as ex:
            self.get_logger().info(f"JointTrajectory generation failed: {ex}")
            
        self.positions = points[0][1].copy()
        self.velocities = points[0][2].copy()
        
        self._eval_thread = evaluate_trajectory_threaded(self.trajectory, self.manipulator, self.obstacles)
        
        
        #self.state = self.SIMULATING

    def _load_environment(self, filename: Path) -> None:
        self.get_logger().info(str(filename))
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
            attempt_max=30
        )

        self._reset_robot_state()
        self._cost_map_cache = self._create_cost_map(
            self.get_clock().now().to_msg()
        )
        self._publish_robot_description()
        self._restart_moveit()
        
        if self.manipulator.in_collision(self.start_config) or self.manipulator.in_collision(self.goal_config):
            self._pending_goal_error = "Failed to generate a random environment with valid start or goal configuration"

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
            
            # Remove the custom <trajsim2d> element and all its contents.
            robot_description = re.sub(
                r"<trajsim2d\b[^>]*>.*?</trajsim2d\s*>",
                "",
                self.description,
                flags=re.DOTALL,
            )
            
            message = String()
            message.data = robot_description
            self.robot_description_pub.publish(message)
            semantic_robot_description  = self._generate_srdf(robot_description)
            message = String()
            message.data = semantic_robot_description
            self.semantic_robot_description_pub.publish(message)

            # Also expose it as a ROS 2 parameter.
            self.set_parameters([
                rclpy.parameter.Parameter(
                    "robot_description",
                    rclpy.Parameter.Type.STRING,
                    robot_description,
                )
            ])
        except Exception as ex:
            self._pending_goal_error(f"Failed to publish robot description: {ex}")
        finally:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)

    def _restart_moveit(self,refresh_config=True,pipeline_id="ompl") -> None:
        self.get_logger().info("Restarting MoveIt")

        if refresh_config:
            self.config_dir = self._generate_moveit_config()

        if not self._relaunch_moveit_client.wait_for_service(timeout_sec=5.0):
            self._pending_goal_error = "MoveIt restart service is unavailable"

        request = RestartMoveit.Request()
        request.foldername = str(self.config_dir)
        request.pipeline_id = str(pipeline_id)
        self.pipeline_id = pipeline_id
        
        try:
            response = self._relaunch_moveit_client.call(request)

            if response is None or not response.result:
                self._pending_goal_error = f"MoveIt restart failed with: {response}"

            self._initialise_moveit_interfaces()

        except Exception as e:
            self._pending_goal_error = f"Failed to restart MoveIt: {e}"
        
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
        srdf = self._generate_srdf(self.description)

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
            "kinematics.yaml"
        ):
            (config_dir / filename).unlink(missing_ok=True)

        # Remove all files matching *_planning.yaml
        for filepath in config_dir.glob("*_planning.yaml"):
            filepath.unlink(missing_ok=True)
            
            
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
        kinematics = {
            "manipulator": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_search_resolution": "0.005",
                "kinematics_solver_timeout": "0.05"}}
        
        with (config_dir / "kinematics.yaml").open("w") as f:
            yaml.safe_dump(kinematics, f,sort_keys=False)

        # Setup all configuration file options
        for plugin,planner_config in self.PLANNER_CONFIGS.items():
            
            joint_list = self._get_joint_list(self.manipulator.n)
            
            planner_config = {
                "planning_plugin": plugin,

                "request_adapters": (
                    "default_planner_request_adapters/AddTimeOptimalParameterization "
                    "default_planner_request_adapters/ResolveConstraintFrames "
                    "default_planner_request_adapters/FixWorkspaceBounds "
                    "default_planner_request_adapters/FixStartStateBounds "
                    "default_planner_request_adapters/FixStartStateCollision "
                    "default_planner_request_adapters/FixStartStatePathConstraints"
                ),

                "start_state_max_bounds_error": 0.1,

                "planner_configs": planner_config,

                "manipulator": {
                    "planner_configs": list(planner_config.keys()),
                    "projection_evaluator": f"joints({joint_list})",
                },
            }
            
            for plugin_id in self.PLUGIN_LIST:
                if plugin.startswith(plugin_id):
                    with (config_dir / (plugin_id +"_planning.yaml")).open("w") as f:
                        yaml.safe_dump(planner_config, f, sort_keys=False)
                    break

        self.get_logger().info(
            f"Generated MoveIt configuration in {config_dir}"
        )

        return config_dir

    def _generate_srdf(self,urdf: str) -> str:
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
        
        ET.SubElement(
            srdf,
            "virtual_joint",
            {
                "name": "world_joint",
                "type": "fixed",
                "parent_frame": "map",
                "child_link": base_link,
            },
        )

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
    
    def _get_joint_list(self,n:int) -> str:
        joint_string = "joint_"
        joint_list = ""
        for i in range(n-1):
            joint_list = joint_list + joint_string + str(i+1) + ","
        joint_list = joint_list + joint_string + str(n)
        
        return joint_list
    
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
        request = goal_handle.request
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
            if self._moveit_error is not None:
                self._pending_goal_error = "MOVEIT ERROR: " + self._moveit_error
                self.state = self.IDLE
                self._pending_goal = None
            elif str(self.pipeline_id) != str(self._pending_goal.request.pipeline_id):
                self._restart_moveit(False,self._pending_goal.request.pipeline_id)
            elif self._planning_scene is None:
                self.state = self.GENERATING_PLANNING_SCENE
                self._planning_scene = self._generate_planning_scene()
            elif self._planning_scene_set is None:
                self.state = self.SETTING_PLANNING_SCENE
                self._set_planning_scene(self._planning_scene)
            elif self._moveit_goal_started is None:
                self.state = self.INITIALISING_MOVEIT_GOAL
                self._start_moveit_goal(self._pending_goal)
            elif self._moveit_goal_complete is None:
                if self._moveit_goal_feedback is not None:
                    self.state = "MOVEIT_" + self._moveit_goal_feedback
            elif self._moveit_goal_complete is True:
                if self._moveit_result is not None:
                    self.state = self.CONVERTING_TRAJECTORY
                    self.trajectory = self._convert_trajectory(
                        self._moveit_result
                    )
                    self.state = self.SIMULATING
                else:
                    if self._moveit_error is not None:
                        self._pending_goal_error = (
                            "MOVEIT_ERROR_" + self._moveit_error
                        )
                    else:
                        self._pending_goal_error = (
                            "UNKNOWN_ERROR_NO_TRAJECTORY"
                        )
                    self.state = self.IDLE
                    
                self._pending_goal = None
            else:
                self._pending_goal = None
                
        if self._moveit_goal_started and not self._moveit_goal_complete:
            self._moveit_cancel_goal()
        self._planning_scene = None
        self.get_logger().info(f"Planning Scene: {self._planning_scene}")
        self._planning_scene_set = None
        self._moveit_goal_started = None
        self._moveit_goal_complete = None
        self._moveit_result = None
        self._moveit_error = None
        
    def _generate_planning_scene(self) -> PlanningScene:
        
        planning_scene = PlanningScene()
        try:
            if self._cost_map_cache is None:
                raise Exception ("No costmap available")
            
            planning_scene.robot_state = self._get_robot_state_msg()
            planning_scene.robot_model_name = "trajsim2d_manipulator"
            planning_scene.is_diff = False
            
            planning_scene.world = self._costmap2planningworld(self._cost_map_cache)
            
            self._print_planning_scene(planning_scene,10)
        except Exception as ex:
            self.get_logger().error(f"Failed to Create Planning Scene: {ex}")
            self._moveit_error(f"Failed to Create Planning Scene: {ex}")
        return planning_scene
            
            
            
        
        convex_border = decompose_to_convex_shapes(self.border)
        scene_polygons = [
            polygon
            for polygon in [*convex_border, *self.obstacles]
            if polygon is not None
        ]

        try:
            planning_scene.robot_state = self._get_robot_state_msg()
            planning_scene.robot_model_name = "trajsim2d_manipulator"
            planning_scene.is_diff = False

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
                object_color = ObjectColor()
                object_color.id = object_id
                object_color.color = self.COLORS["blue"]
                planning_scene.object_colors.append(object_color)

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
            
            self._print_planning_scene(planning_scene,20)
        except Exception as ex:
            self.get_logger().error(f"Failed to Create Planning Scene: {ex}")
            self._moveit_error(f"Failed to Create Planning Scene: {ex}")

        return planning_scene
    
    def _print_planning_scene(self,msg:PlanningScene,severity:int) -> None:
        """Log the contents of a PlanningScene message."""

        logger = self.get_logger()

        logger.log("PlanningScene:", severity)
        logger.log(f"  name: {msg.name}", severity)
        logger.log(f"  robot_model_name: {msg.robot_model_name}", severity)
        logger.log(f"  is_diff: {msg.is_diff}", severity)

        # Robot state
        logger.log("  robot_state:", severity)
        logger.log(
            f"    joint_state: {msg.robot_state.joint_state}",
            severity
        )
        logger.log(
            f"    multi_dof_joint_state: "
            f"{msg.robot_state.multi_dof_joint_state}",
            severity
        )
        logger.log(
            f"    attached_collision_objects: "
            f"{msg.robot_state.attached_collision_objects}",
            severity
        )

        # Fixed transforms
        logger.log("  fixed_frame_transforms:", severity)
        for transform in msg.fixed_frame_transforms:
            logger.log(f"    {transform}", severity)

        # Collision matrix
        logger.log(
            f"  allowed_collision_matrix: "
            f"{msg.allowed_collision_matrix}",
            severity
        )

        # Link properties
        logger.log(f"  link_padding: {msg.link_padding}", severity)
        logger.log(f"  link_scale: {msg.link_scale}", severity)

        # Object colors
        logger.log("  object_colors:", severity)
        for color in msg.object_colors:
            logger.log(f"    {color}", severity)

        # World geometry
        logger.log(
            f"  world.collision_objects: "
            f"{msg.world.collision_objects}",
            severity
        )
        logger.log(
            f"  world.octomap: {msg.world.octomap}",
            severity
        )
    
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
    
    def _triangulate_polygon(self, polygon: np.ndarray) -> tuple:
        
        
        polygon = np.asarray(polygon, dtype=float)
        
        if polygon.ndim != 2 or polygon.shape[1] != 2:
            self.get_logger().warn("Polygon must have shape (N, 2)")

        if not np.all(np.isfinite(polygon)):
            self.get_logger().warn("Polygon contains non-finite coordinates")

        # Remove duplicated closing vertex
        if len(polygon) > 1 and np.allclose(polygon[0], polygon[-1]):
            polygon = polygon[:-1]

        # Remove consecutive duplicate vertices
        if len(polygon) > 1:
            keep = np.ones(len(polygon), dtype=bool)
            keep[1:] = np.linalg.norm(np.diff(polygon, axis=0), axis=1) > 1e-10
            polygon = polygon[keep]

        if len(polygon) < 3:
            self.get_logger().warn("Polygon needs at least 3 distinct vertices")

        # Signed area
        area = 0.5 * np.sum(
            polygon[:, 0] * np.roll(polygon[:, 1], -1)
            - polygon[:, 1] * np.roll(polygon[:, 0], -1)
        )

        if abs(area) < 1e-12:
            self.get_logger().warn("Polygon has zero area")

        # Ensure counter-clockwise winding
        if area < 0:
            polygon = polygon[::-1].copy()

        # Fan triangulation: convex polygons only
        triangles = [
            (0, i, i + 1)
            for i in range(1, len(polygon) - 1)
        ]

        return polygon, triangles
    
    def _costmap2planningworld(self, costmap: OccupancyGrid) -> PlanningSceneWorld:
        planning_world = PlanningSceneWorld()
        
        width = costmap.info.width
        height = costmap.info.height
        resolution = costmap.info.resolution
        origin = costmap.info.origin

        if width == 0 or height == 0 or resolution <= 0.0:
            return planning_world

        # 2D homogeneous transform: base_frame -> map.
        T_map_base = np.asarray(
            self.base_transform, dtype=float
        ).reshape(3, 3)

        # Inverse transform: map -> base_frame.
        T_base_map = np.linalg.inv(T_map_base)

        data = costmap.data
        occupied_threshold = 50

        # Box height and centre in the planning scene.
        z_min = float(self._moveit_scene_z_min)
        z_max = float(self._moveit_scene_z_max)

        if z_max <= z_min:
            raise ValueError(
                f"Invalid planning scene Z limits: "
                f"z_min={z_min}, z_max={z_max}"
            )

        box_height = z_max - z_min
        box_center_z = (z_min + z_max) / 2.0

        # Costmap origin yaw.
        yaw_origin = 2.0 * np.arctan2(
            origin.orientation.z,
            origin.orientation.w,
        )

        c = np.cos(yaw_origin)
        s = np.sin(yaw_origin)
        R_map_grid = np.array([
            [c, -s],
            [s,  c],
        ])

        # Rotation from map coordinates into base_frame.
        R_base_map = T_base_map[:2, :2]

        # Transform the costmap origin's orientation into base_frame.
        R_base_grid = R_base_map @ R_map_grid
        yaw_base_grid = np.arctan2(
            R_base_grid[1, 0],
            R_base_grid[0, 0],
        )

        qz = float(np.sin(yaw_base_grid / 2.0))
        qw = float(np.cos(yaw_base_grid / 2.0))

        # OccupancyGrid data is row-major: index = y * width + x.
        occupied = [
            data[y * width + x] >= occupied_threshold
            for y in range(height)
            for x in range(width)
        ]

        # Merge contiguous occupied cells into horizontal runs.
        for y in range(height):
            x = 0

            while x < width:
                if not occupied[y * width + x]:
                    x += 1
                    continue

                start_x = x

                while x < width and occupied[y * width + x]:
                    x += 1

                run_width = x - start_x

                # Centre of this run in the costmap grid coordinates.
                p_grid = np.array([
                    (start_x + run_width / 2.0) * resolution,
                    (y + 0.5) * resolution,
                    1.0,
                ])

                # Convert grid position into map coordinates.
                # Include the costmap origin translation.
                p_map = np.array([
                    origin.position.x,
                    origin.position.y,
                ]) + R_map_grid @ p_grid[:2]

                p_map_h = np.array([
                    p_map[0],
                    p_map[1],
                    1.0,
                ])

                # Convert map position into base_frame coordinates.
                p_base = T_base_map @ p_map_h

                collision = CollisionObject()
                collision.header = deepcopy(costmap.header)
                collision.header.frame_id = self.base_frame
                collision.id = f"costmap_{start_x}_{y}"

                box = SolidPrimitive()
                box.type = SolidPrimitive.BOX
                box.dimensions = [
                    run_width * resolution,
                    resolution,
                    box_height,
                ]

                pose = Pose()
                pose.position.x = float(p_base[0])
                pose.position.y = float(p_base[1])
                pose.position.z = box_center_z

                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = qz
                pose.orientation.w = qw

                collision.primitives.append(box)
                collision.primitive_poses.append(pose)
                collision.operation = CollisionObject.ADD

                planning_world.collision_objects.append(collision)

        return planning_world
    
    def _set_planning_scene(
        self,
        planning_scene: PlanningScene,
    ) -> None:

        if self._planning_scene_request_pending:
            return

        if not self._planning_scene_client.service_is_ready():
            if not self._planning_scene_client.wait_for_service(
                timeout_sec=1.0
            ):
                self._moveit_error = "Planning_Scene_Server_Not_Available"
                return

        service_request = ApplyPlanningScene.Request()
        service_request.scene = planning_scene

        self._planning_scene_set = None
        self._planning_scene_request_pending = True

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
            self._moveit_error = str(exc)
            self._fail_moveit_request(
                f"PLANNING_SCENE_REQUEST_FAILED: {exc}"
            )
    
    def _planning_scene_response_callback(self, future) -> None:
        self._planning_scene_request_pending = False

        try:
            response = future.result()

            if response is not None and response.success:
                self._planning_scene_set = True
                self._moveit_error = None
                self.get_logger().info(
                    "MoveIt planning scene applied successfully"
                )
                return

            self._moveit_error = (
                "ApplyPlanningScene returned success=False"
            )

        except Exception as exc:
            self._moveit_error = str(exc)

        self._planning_scene_set = False

        self._fail_moveit_request(
            "PLANNING_SCENE_FAILED: "
            + str(self._moveit_error)
        )
    
    def _print_move_group_goal(self,msg:MoveGroup.Goal,severity:int) -> None:
        """Print the MoveGroup goal details."""
        self.get_logger().log("MoveGroup Goal:",severity)

        self.get_logger().log(f"Request: {msg.request}",severity)
        self.get_logger().log(f"Planning options: {msg.planning_options}",severity)
    
    def _start_moveit_goal(self, request:MoveitRequest.Goal) -> None:

        
        if self._moveit_goal_request_pending:
            return

        if self._moveit_goal_started is True:
            return

        if self._planning_scene_set is not True:
            self.get_logger().warn(f"PLANNING SCENE NOT SET")
            self._moveit_error = "PLANNING SCENE NOT SET"
            return

        if not self._moveit_action_client.server_is_ready():
            self.get_logger().warn(f"MOVEIT ACTION SERVER UNAVAILABLE")
            self._moveit_goal_started = False
            self._moveit_goal_complete = True
            self._moveit_error = "ACTION_SERVER_UNAVAILABLE"
            return

        self.save_location = request.foldername
        self.foldername = Path(self.save_location) / Path(time.strftime("%d_%m_%y_%H_%M_%S"))

        try:
            motion_plan_goal = MoveGroup.Goal()
            
            motion_plan_request = MotionPlanRequest()
            
            
            # variable planner inputs
            motion_plan_request.pipeline_id = request.request.pipeline_id
            motion_plan_request.planner_id = request.request.planner_id
            motion_plan_request.num_planning_attempts = request.request.num_planning_attempts
            motion_plan_request.allowed_planning_time = request.request.allowed_planning_time
            
            # default settings
            # path_constraints = Constraints()
            # motion_plan_request.path_constraints = path_constraints
            # trajectory_constraints = TrajectoryConstraints()
            # motion_plan_request.trajectory_constraints = trajectory_constraints
            motion_plan_request.max_velocity_scaling_factor = 1.0
            motion_plan_request.max_acceleration_scaling_factor = 1.0
            # motion_plan_request.workspace_parameters = WorkspaceParameters # from moveit_msgs.msg import WorkspaceParameters
            
            # set planner inputs
            motion_plan_request.start_state = self._get_robot_state_msg()
            
            if not request.request.goal_constraints:
                goal_tolerance = self.goal_tolerance
            else:
                goal_tolerance = request.request.goal_constraints[0].joint_constraints[0].tolerance_above
                
            motion_plan_request.goal_constraints = self._generate_goal_constraints(motion_plan_request.start_state.joint_state.name,goal_tolerance)
            motion_plan_request.group_name = "manipulator"
            
            motion_plan_goal.request = motion_plan_request

            planning_options = PlanningOptions()
            # The scene has already been applied using ApplyPlanningScene.
            planning_options.planning_scene_diff.is_diff = True
            planning_options.plan_only = True
            planning_options.look_around = False
            #planning_options.look_around_attempts = 0
            #planning_options.max_safe_execution_cost = 0.0
            planning_options.replan = False
            #planning_options.replan_attempts = 1
            #planning_options.replan_delay = 0.0


            motion_plan_goal.planning_options = planning_options
            
            self._print_move_group_goal(motion_plan_goal,10)

            self._moveit_goal_request_pending = True
            self._moveit_goal_started = None
            self._moveit_goal_complete = None
            self._moveit_goal_feedback = None
            self._moveit_error = None
            self._moveit_goal_handle = None
            self._moveit_result = None
            self._moveit_request = motion_plan_goal.request

            future = self._moveit_action_client.send_goal_async(
                motion_plan_goal,
                feedback_callback=self._moveit_feedback_callback,
            )

            self._moveit_goal_future = future
            future.add_done_callback(
                self._moveit_goal_response_callback
            )

        except Exception as exc:
            self.get_logger().warn(f"************************************")
            self.get_logger().warn(f"ERROR PROCESSING MOVEIT GOAL: {exc}")
            self.get_logger().warn(f"************************************")
            self._moveit_goal_request_pending = False
            self._moveit_goal_started = False
            self._moveit_goal_complete = True
            self._moveit_error = f"GOAL_SEND_FAILED: {exc}"
    
    def _generate_goal_constraints(self, names: list[str], tolerance:float) -> list[Constraints]:
        
        goal_constraints = []
        goal_constraint = Constraints()
        joint_constraints = []
        
        for joint_name,position in zip(names,self.goal_config*-1):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = position
            joint_constraint.tolerance_above = tolerance
            joint_constraint.tolerance_below = tolerance
            joint_constraints.append(joint_constraint)
            
        goal_constraint.joint_constraints = joint_constraints
        
        ## Empty 
        # position_constraints = []
        # position_constraint = PositionConstraint() # from moveit_msgs.msg import
        # position_constraints.append(position_constraint)
        # goal_constraint.position_constraints = position_constraints
        # orientation_constraints = []
        # orientation_constraint = OrientationConstraint() # from moveit_msgs.msg import
        # orientation_constraints.append(orientation_constraint)
        # goal_constraint.orientation_constraints = orientation_constraints
        # visibility_constraints = []
        # visibility_constraint = VisibilityConstraint() # from moveit_msgs.msg import
        # visibility_constraints.append(visibility_constraint)
        # goal_constraint.visibility_constraints = visibility_constraints
        
        goal_constraints.append(goal_constraint)
        
        return goal_constraints
    
    def _moveit_goal_response_callback(self, future) -> None:
        self._moveit_goal_request_pending = False

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._moveit_goal_started = False
            self._moveit_goal_complete = True
            self._moveit_error = f"GOAL_RESPONSE_FAILED: {exc}"
            return

        if not goal_handle.accepted:
            self._moveit_goal_started = False
            self._moveit_goal_complete = True
            self._moveit_error = "GOAL_REJECTED"
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

            self.save_moveit_performance_to_file(self.foldername,result,self._moveit_request)
            self._moveit_request = None

            if (
                result.error_code.val
                != MoveItErrorCodes.SUCCESS
            ):
                self._moveit_error = (
                    f"PLANNING_FAILED_{result.error_code.val}"
                )
                self._moveit_result = None
                self._moveit_goal_complete = True
                return

            self._moveit_result = result
            self._moveit_goal_complete = True

        except Exception as exc:
            self.get_logger().warn(f"************************************")
            self.get_logger().warn(f"ERROR PROCESSING MOVEIT RESULT: {exc}")
            self.get_logger().warn(f"************************************")
            self._moveit_goal_complete = True
            self._moveit_error = f"RESULT_FAILED: {exc}"
            self._moveit_result = None
    
    def _convert_trajectory(self,moveit_result:MoveGroup.Result) -> Trajectory:
        moveit_trajectory = moveit_result.planned_trajectory.joint_trajectory
        self._trajectory_callback(moveit_trajectory)
        return self.trajectory
    
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
        self._pending_goal = None
    
    def _main_loop(self) -> None:
        """Process one requested action and return to a passive state."""
        if self._pending_goal is None:
            # Visualise
            if self.state == self.SIMULATING:
                self._advance_trajectory()
            self._refresh_visualisation()
            return

        request = self._pending_goal
        
        if isinstance(request, MoveitRequest.Goal):
            try:
                self._get_moveit_trajectory()
            except (OSError, ValueError, RuntimeError) as error:
                self.get_logger().warn(f"************************************")
                self.get_logger().warn(f"MOVEIT GOAL FAILED: {error}")
                self.get_logger().warn(f"************************************")
                self._pending_goal_error = str(error)
            finally:
                self._pending_goal = None
                self._goal_complete.set()
                return
        
        try:
            if request.goal == Simulation.Goal.GET_CURRENT_STATE:
                self._pending_goal = None
                self._goal_complete.set()
                return
        except Exception as error:
            self.get_logger().warn(f"************************************")
            self.get_logger().warn(f"SIMULATION GOAL FAILED: {error}")
            self.get_logger().warn(f"************************************")
            self._pending_goal_error = str(error)
            
        
        try:
            self.state = self._goal_state(request.goal)
            if request.goal == Simulation.Goal.GENERATE_RANDOM_ENVIRONMENT:
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
        except Exception as error:
            self.get_logger().warn(f"************************************")
            self.get_logger().warn(f"SIMULATION GOAL FAILED: {error}")
            self.get_logger().warn(f"************************************")
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

    def _get_moveit_robot_state_msg(self,now:Time = None,names: list = None) -> DisplayRobotState:
        display_robot_state = DisplayRobotState()
        robot_state_msg = self._get_robot_state_msg(now,names)
        display_robot_state.state = robot_state_msg
        return display_robot_state
    
    def _get_robot_state_msg(self,now:Time = None,names: list = None) -> RobotState:
            
            state = RobotState()
            joint_state = self._get_joint_state_msg(now,names)
            state.joint_state = joint_state
            return state
        
    def _get_joint_state_msg(self,now:Time = None,names: list = None) -> JointState:
        
        if now is None:
            now = self.get_clock().now().to_msg()
        if names is None:
            names = [f"joint_{index + 1}" for index in range(self.manipulator.n)]
        
        state = JointState()
        state.header.stamp = now
        state.header.frame_id = self.base_frame
        state.name = names
        state.position = (self.positions*-1.0).tolist()
        state.velocity = (self.velocities*-1.0).tolist()
        state.effort = (self.efforts*-1.0).tolist()
        return state
                
    def _interrupt_loop_sim(self) -> None:
        """Advance simulation and publish ROS/visualisation outputs."""
        if (self.state is not self.IDLE and self.state is not self.SIMULATING) or self.manipulator is None or self.positions is None or self.velocities is None:
            return
        now = self.get_clock().now().to_msg()
        names = [f"joint_{index + 1}" for index in range(self.manipulator.n)]
        state = self._get_joint_state_msg(now,names)
        self.robot_state_pub.publish(state)
        display_state = self._get_moveit_robot_state_msg()
        self.display_robot_state_pub.publish(display_state)        

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
        
        if done or not self.display_trajectory:
            # check if calculation thread still running 
            if self._eval_thread.is_alive():
                pass
            else:
                # save trajectory
                save_trajectory_to_file(foldername=self.foldername,trajectory=self.trajectory,manip=self.manipulator)
                self._reset_robot_state()
                self.trajectory = None
                self.canvas.remove_shape(self.arm_ids)
                self.arm_ids = [None]
                self.state = self.IDLE
        else:
            [self.positions,self.velocities,self.efforts] = calculate_robot_state(self.manipulator,self.trajectory,index)
            
        return

    def save_moveit_performance_to_file(
        self,
        foldername: Path,
        moveit_result: MoveGroup.Result,
        moveit_planner: MotionPlanRequest
    ) -> None:
        
        foldername.mkdir(parents=True, exist_ok=True)

        performance_data = {
            "timestamp": time.strftime("%d_%m_%y_%H_%M_%S"),
            "result": {
                "success": moveit_result.error_code.val == 1,
                "error_code": moveit_result.error_code.val,
                "planning_time": moveit_result.planning_time,
            },
            "metadata": {
                "pipeline_id": moveit_planner.pipeline_id,
                "planner_id": moveit_planner.planner_id,
                "group_name": moveit_planner.group_name,
                "allowed_planning_time": moveit_planner.allowed_planning_time,
                "num_planning_attempts": moveit_planner.num_planning_attempts,
                "goal_constraints": [
                    message_to_ordereddict(constraint)
                    for constraint in moveit_planner.goal_constraints
                ],
                "trajectory_constraints": message_to_ordereddict(
                    moveit_planner.trajectory_constraints
                ),
            },
        }

        filename = foldername / "moveit_performance.yaml"

        performance_data = self._to_plain_data(performance_data)
        
        with filename.open("w", encoding="utf-8") as file:
            yaml.safe_dump(
                performance_data,
                file,
                sort_keys=False,
                allow_unicode=True,
            )

    def _to_plain_data(self,value):
        """Recursively convert mappings and sequences to YAML-safe Python types."""
        if isinstance(value, Mapping):
            return {
                key: self._to_plain_data(item)
                for key, item in value.items()
            }

        if isinstance(value, (list, tuple)):
            return [self._to_plain_data(item) for item in value]

        return value
    
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
