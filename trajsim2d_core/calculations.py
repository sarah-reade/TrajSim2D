###############################################################################
## @file calculations.py
## @brief 
##
## This file is part of the TrajSim2D project, a 2D planar manipulator simulator
## for trajectory planning, collision testing, and environment visualization.
## 
## Module responsibilities:
## - <List main purpose of this file, e.g., "Forward kinematics and Jacobian calculations">
## - <Optional: physics, dynamics, or utility functions>
##
## Author: Sarah Reade
## Email: 28378329@students.lincoln.ac.uk
## Date: 2025-10-23
## Version: 0.0.1
##
## License: MIT
##
## Usage:
## >>> from trajsim2d_core.manip import PlanarManipulator
## >>> arm = PlanarManipulator(num_joints=3, link_lengths=[1.0, 1.0, 0.5])
## >>> q = [0.0, 1.0, 0.5]
## >>> pos = arm.forward_kinematics(q)
###############################################################################

# Imports
from utils import calc_array_diff

# calculate outputs based on trajectory
def evaluate_trajectory(traj):
    ## for all timepoints

    # calculate dt
    dt = calc_array_diff(traj.time)

    # calculate qdot

    # calculate qdotdot

    # calculate torque

    # calculate base wrench force

    return 

# 


# calculate torque
def calculate_torque():
    return calculate_static_torque()

# calculate static torque
def calculate_static_torque():
    return

# calculate base wrench force
def calculate_base_wrench_force():
    """
    ## @brief Computes the base wrench for of the manipulator
    ##
    ## This function calculates the base wrench force of the manipulator, based on the
    ## the parameters of the manipulator and torque of the joints
    ##
    ## @param manip 2dManip: Manipulator object (supplying: gravity in base frame, pose 
    ## of each link, mass of each link, Jacobian, and torque).
    ## @param q List[float]: Joint angles [q1, q2, ..., qn] in radians.
    ## @return Tuple[float, float, float]: (Zx, Zy, theta) .
    """
    pass


