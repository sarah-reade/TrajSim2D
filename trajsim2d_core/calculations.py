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
from trajsim2d_core.utils import calc_array_diff
from trajsim2d_core.twodmanip import PlanarManipulator
import numpy as np

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
def calculate_torque(manip: PlanarManipulator, base_tf, q, qdot = None, qdotdot = None):
    """
    ## @brief Computes the torque for each joint of the manipulator
    ##
    ## This function calculates the torque required at each joint of the manipulator, based on the
    ## the parameters of the manipulator and the joint positions, velocities, and accelerations.
    ##
    ## @param manip 2dManip: Manipulator object (supplying: gravity in base frame, pose 
    ## of each link, mass of each link, Jacobian, and torque).
    ## @param q List[float]: Joint angles [q1, q2, ..., qn] in radians.
    ## @param qdot List[float] or None: Joint velocities [q1_dot, q2_dot, ..., qn_dot] in radians/sec.
    ## @param qdotdot List[float] or None: Joint accelerations [q1_dotdot, q2_dotdot, ..., qn_dotdot] in radians/sec^2.
    """
    if base_tf.shape != (3, 3):
        raise ValueError("base_tf must be a 3x3 transformation matrix.")

    if len(q) != manip.num_joints:
        raise ValueError(f"Length of q ({len(q)}) must match number of joints ({manip.num_joints}).")
    
    if qdot is None and qdotdot is None:
        return calculate_static_torque(manip,base_tf, q)
    else:
        raise NotImplementedError("Dynamic torque calculation not implemented yet.")    

def calculate_static_torque(manip: PlanarManipulator, base_tf, q):
    """
    ## @brief Computes the static torque for each joint of the manipulator
    ##
    ## This function calculates the static torque required at each joint of the manipulator, based on the
    ## the parameters of the manipulator and the joint positions.
    ##
    ## @param manip 2dManip: Manipulator object (supplying: gravity in base frame, pose 
    ## of each link, mass of each link, Jacobian, and torque).
    ## @param q List[float]: Joint angles [q1, q2, ..., qn] in radians.
    """
    if base_tf.shape != (3, 3):
        raise ValueError("base_tf must be a 3x3 transformation matrix.")
    
    if len(q) != manip.n:
        raise ValueError(f"Length of q ({len(q)}) must match number of joints ({manip.n}).")
    
    # Tau array
    tau = np.zeros(manip.n)
    
    # gravity constant
    g = -9.81
    
    # Get joint to joint transforms
    joint_tfs = manip.forward_kinematics(base_tf, q)
    
    
    # For each joint (starting from the first joint)
    for i in range(0, manip.n):
        
        tau_ij = 0.0
        
        # For each joint to the end effector after this joint
        for j in range(i, manip.n):
            
            
            # Calculate Gravity force of link j in frame i
            m = manip.link_masses[j]
            G = np.array([0, m * g]) 
            V = joint_tfs[j+1][0:2,2] - joint_tfs[i][0:2,2]
            
            
            # Step 1: compute rotation angle
            alpha = np.arctan2(V[1], V[0])

            # Step 2: rotation matrix
            R = np.array([
                [np.cos(alpha), -np.sin(alpha)],
                [np.sin(alpha),  np.cos(alpha)]
            ])

            # Step 3: rotate G into V-frame
            G_rotated = R @ G  # @ is matrix multiplication in numpy
            
            # Step 4: compute torque contribution
            d = np.linalg.norm(V)
            
            tau_ij = G_rotated[1]  * d
            
            tau[i] += tau_ij
                
        
    return tau

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


