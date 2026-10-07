###############################################################################
## @file file_parser.py
## @brief 
##
## This file is part of the TrajSim2D project, a 2D planar manipulator simulator
## for trajectory planning, collision testing, and environment visualization.
## 
## Module responsibilities:
## - Save data to files
##
## Author: Sarah Reade
## Email: 28378329@students.lincoln.ac.uk
## Date: 2026-01-27
## Version: 0.0.1
##
## License: MIT
##
## Usage:
## >>> from trajsim2d_core.file_parser import save_trajectory_to_file
###############################################################################

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional
from trajsim2d_core.calculations import Trajectory
from trajsim2d_core.twodmanip import PlanarManipulator
import numpy as np
import os


@dataclass
class CanvasData:
    """Persistent data needed to reconstruct a TrajSim2D visualisation."""

    arm: PlanarManipulator
    border: Optional[np.ndarray]
    obstacles: List[np.ndarray]
    base_transform: np.ndarray
    start_config: Optional[np.ndarray]
    end_config: Optional[np.ndarray]
    arm_urdf: str


def _array_to_json(value):
    if value is None:
        return None
    return np.asarray(value, dtype=float).tolist()


def _value_to_json(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def _load_array(value, name, ndim=None):
    if value is None:
        return None
    array = np.asarray(value, dtype=float)
    if ndim is not None and array.ndim != ndim:
        raise ValueError(f"{name} must have {ndim} dimensions")
    return array


def save_canvas_to_file(
    filename,
    manip: PlanarManipulator,
    border=None,
    obstacles=None,
    base_transform=None,
    start_config=None,
    end_config=None,
    arm_filename=None,
):
    """
    Save a complete visualisation scene and its manipulator URDF.

    The ``.canvas`` file stores scene data as JSON. The arm remains in the
    existing URDF format and is saved alongside the canvas unless an explicit
    ``arm_filename`` is supplied. Paths stored in the canvas are relative to
    the canvas file, so the pair can be moved together.

    @param filename Canvas JSON filename.
    @param manip Manipulator to save to URDF.
    @param border Nx2 boundary vertices.
    @param obstacles List of Nx2 obstacle polygons.
    @param base_transform 3x3 active scene transform; defaults to manip.base_tf.
    @param start_config Initial arm joint configuration.
    @param end_config Final arm joint configuration.
    @param arm_filename Optional URDF path. Defaults to the canvas stem + '.urdf'.
    """
    canvas_path = Path(filename)
    canvas_path.parent.mkdir(parents=True, exist_ok=True)
    urdf_path = (
        Path(arm_filename)
        if arm_filename is not None
        else canvas_path.with_suffix(".urdf")
    )
    if not urdf_path.is_absolute():
        urdf_path = canvas_path.parent / urdf_path
    urdf_path.parent.mkdir(parents=True, exist_ok=True)

    scene_base_tf = (
        np.asarray(manip.base_tf, dtype=float)
        if base_transform is None
        else np.asarray(base_transform, dtype=float)
    )
    if scene_base_tf.shape != (3, 3):
        raise ValueError("base_transform must have shape (3, 3)")

    scene_border = _load_array(border, "border", ndim=2)
    if scene_border is not None and scene_border.shape[1] != 2:
        raise ValueError("border must have shape (N, 2)")

    scene_obstacles = []
    for index, obstacle in enumerate(obstacles or []):
        obstacle_array = _load_array(obstacle, f"obstacles[{index}]", ndim=2)
        if obstacle_array.shape[1] != 2:
            raise ValueError(f"obstacles[{index}] must have shape (N, 2)")
        scene_obstacles.append(obstacle_array)

    for name, config in (("start_config", start_config), ("end_config", end_config)):
        if config is not None and np.asarray(config).size != manip.n:
            raise ValueError(f"{name} must contain {manip.n} joint values")

    # Keep the URDF's existing base transform consistent with the saved scene.
    manip.base_tf = scene_base_tf.copy()
    manip.save_to_urdf(str(urdf_path))

    relative_urdf = os.path.relpath(urdf_path, canvas_path.parent)
    data = {
        "format": "trajsim2d.canvas",
        "version": 1,
        "arm_urdf": relative_urdf,
        "border": _array_to_json(scene_border),
        "obstacles": [_array_to_json(obstacle) for obstacle in scene_obstacles],
        "base_transform": _array_to_json(scene_base_tf),
        "start_config": _array_to_json(start_config),
        "end_config": _array_to_json(end_config),
    }
    with canvas_path.open("w", encoding="utf-8") as file:
        json.dump(data, file, indent=2)


def load_canvas_from_file(filename):
    """
    Load a visualisation scene and its associated manipulator URDF.

    @return CanvasData containing the loaded arm and scene arrays.
    """
    canvas_path = Path(filename)
    with canvas_path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    if data.get("format") != "trajsim2d.canvas":
        raise ValueError("File is not a TrajSim2D canvas file")
    if data.get("version") != 1:
        raise ValueError(f"Unsupported canvas version: {data.get('version')}")

    arm_reference = data.get("arm_urdf")
    if not arm_reference:
        raise ValueError("Canvas file is missing 'arm_urdf'")
    arm_path = Path(arm_reference)
    if not arm_path.is_absolute():
        arm_path = canvas_path.parent / arm_path
    arm = PlanarManipulator(filename=str(arm_path))

    border = _load_array(data.get("border"), "border", ndim=2)
    if border is not None and border.shape[1] != 2:
        raise ValueError("border must have shape (N, 2)")

    obstacles = []
    for index, obstacle in enumerate(data.get("obstacles", [])):
        obstacle_array = _load_array(obstacle, f"obstacles[{index}]", ndim=2)
        if obstacle_array.shape[1] != 2:
            raise ValueError(f"obstacles[{index}] must have shape (N, 2)")
        obstacles.append(obstacle_array)

    base_transform = _load_array(
        data.get("base_transform"), "base_transform", ndim=2
    )
    if base_transform is None or base_transform.shape != (3, 3):
        raise ValueError("base_transform must have shape (3, 3)")
    arm.base_tf = base_transform.copy()

    start_config = _load_array(data.get("start_config"), "start_config")
    end_config = _load_array(data.get("end_config"), "end_config")
    for name, config in (("start_config", start_config), ("end_config", end_config)):
        if config is not None and config.size != arm.n:
            raise ValueError(f"{name} must contain {arm.n} joint values")

    return CanvasData(
        arm=arm,
        border=border,
        obstacles=obstacles,
        base_transform=base_transform,
        start_config=start_config,
        end_config=end_config,
        arm_urdf=str(arm_path),
    )


def save_trajectory_to_file(foldername, trajectory: Trajectory, manip: PlanarManipulator):
    """
    @brief Save trajectory data to a text file.
    @param foldername Name of the file to save the trajectory.
    @param trajectory Trajectory object containing the data to save.
    """
    # make folder if it doesn't exist
    if not os.path.exists(foldername):
        os.makedirs(foldername)
    
    print("Saving file to: %s", str(foldername) + "/trajectory.csv")
    with open(str(foldername) + "/trajectory.csv", 'w') as f:
        # Write header
        header = ["time"] \
                + [f"q{j}" for j in range(trajectory.q.shape[1])] \
                + [f"qdot{j}" for j in range(trajectory.qdot.shape[1])] \
                + [f"qdotdot{j}" for j in range(trajectory.qdotdot.shape[1])] \
                + [f"tau{j}" for j in range(trajectory.tau.shape[1])] \
                + ["Fx", "Fy", "Mz"] \
                + [
                    "in_collision",
                    "qdotdot_exceeded",
                    "tau_exceeded",
                    "adhesion_exceeded",
                ]
        f.write(",".join(header) + "\n")
        
        # Write data
        for i in range(len(trajectory.time)):
            # save q, qdot, qdotdot, tau, base_wrench, and status flags
            # qdot / qdotdot may not exist for last steps
            qdot_str = ",".join([str(trajectory.qdot[i, j]) for j in range(trajectory.qdot.shape[1])]) \
                        if i < trajectory.qdot.shape[0] else ",".join([""] * trajectory.qdot.shape[1])

            qdotdot_str = ",".join([str(trajectory.qdotdot[i, j]) for j in range(trajectory.qdotdot.shape[1])]) \
                        if i < trajectory.qdotdot.shape[0] else ",".join([""] * trajectory.qdotdot.shape[1])

            line = (
                f"{trajectory.time[i]}," +
                ",".join([str(trajectory.q[i, j]) for j in range(trajectory.q.shape[1])]) + "," +
                qdot_str + "," +
                qdotdot_str + "," +
                ",".join([str(trajectory.tau[i, j]) for j in range(trajectory.tau.shape[1])]) + "," +
                ",".join([str(trajectory.base_wrench[i, j]) for j in range(trajectory.base_wrench.shape[1])]) + "," +
                str(trajectory.in_collision[i]) + "," +
                str(trajectory.qdotdot_exceeded[i]) + "," +
                str(trajectory.tau_exceeded[i]) + "," +
                str(trajectory.adhesion_exceeded[i])
            )
            f.write(line + "\n")
           
    print(trajectory.base_tf)
    print(manip)
    trajectory_metadata = json.dumps({
        "timestamp": time.time(),
        "base_tf": [[float(v) for v in row] for row in trajectory.base_tf.tolist()],
        "manipulator_parameters": {
            "num_links": manip.n,
            "link_widths": _value_to_json(manip.link_width),
            "link_lengths": _value_to_json(manip.link_lengths),
            "joint_radius": _value_to_json(manip.joint_radius),
            "joint_limit_radians": _value_to_json(
                manip.joint_limits.position
            ),
        }
    }, indent=2)
    
    with open(foldername + "/metadata.json", 'w') as f:
        # Write metadata header
        f.write(str(trajectory_metadata))

            
def load_trajectory_from_file(filename) -> Trajectory:
    """
    @brief Load trajectory data from a text file.
    @param filename Name of the file to load the trajectory from.
    @return Trajectory object containing the loaded data.
    """
    
    raise NotImplementedError("Function load_trajectory_from_file is not yet implemented.")
    
    time = []
    q = []
    
    with open(filename, 'r') as f:
        # Skip header
        next(f)
        
        # Read data
        for line in f:
            parts = line.strip().split(',')
            time.append(float(parts[0]))
            q.append([float(val) for val in parts[1:]])
    
    trajectory = Trajectory()
    trajectory.time = np.array(time)
    trajectory.q = np.array(q)
    
    return trajectory
