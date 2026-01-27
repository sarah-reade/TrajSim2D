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
from trajsim2d_core.calculations import Trajectory
from trajsim2d_core.twodmanip import PlanarManipulator
import numpy as np
import os

def save_trajectory_to_file(foldername, trajectory: Trajectory, manip: PlanarManipulator):
    """
    @brief Save trajectory data to a text file.
    @param foldername Name of the file to save the trajectory.
    @param trajectory Trajectory object containing the data to save.
    """
    # make folder if it doesn't exist
    if not os.path.exists(foldername):
        os.makedirs(foldername)
    
    
    with open(foldername + "/trajectory.csv", 'w') as f:
        # Write header
        header = ["time"] \
                + [f"q{j}" for j in range(trajectory.q.shape[1])] \
                + [f"qdot{j}" for j in range(trajectory.qdot.shape[1])] \
                + [f"qdotdot{j}" for j in range(trajectory.qdotdot.shape[1])] \
                + [f"tau{j}" for j in range(trajectory.tau.shape[1])] \
                + ["Fx", "Fy", "Mz"] \
                + ["in_collision"]
        f.write(",".join(header) + "\n")
        
        # Write data
        for i in range(len(trajectory.time)):
            # save q, qdot, qdotdot, tau, base_wrench, in_collision
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
                str(trajectory.in_collision[i])
            )
            f.write(line + "\n")
                
    
    trajectory_metadata = json.dumps({
        "timestamp": time.time(),
        "base_tf": [[float(v) for v in row] for row in trajectory.base_tf.tolist()],
        "manipulator_parameters": {
            "num_links": manip.n,
            "link_widths": manip.link_width,
            "link_lengths": manip.link_lengths,
            "joint_radius": manip.joint_radius,
            "joint_limit_radians": manip.joint_limit
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
