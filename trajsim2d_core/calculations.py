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
from trajsim2d_core.utils import calc_array_diff_array, get_array_midpoints
from trajsim2d_core.twodmanip import PlanarManipulator
import numpy as np

class Path:
    """
    ## @brief Data structure to hold path information
    ##
    ## This class encapsulates the path data for a manipulator, including
    ## joint positions and base transformation matrices at discrete waypoints.
    ##
    ## @param trajectories List[Trajectory]: List of Trajectory objects.
    ## @param base_tfs np.ndarray: Base transformation matrices at each time point.
    ## @param attachment_end bool: Whether the end effector is attached to the base.
    """
    def __init__(self, trajectories, base_tfs, attachment_end):
        self.trajectories = []                  # list of N trajectories
        for trajectory in trajectories:  
            self.trajectories.append(trajectory)
            
        self.base_tfs = base_tfs                # np.ndarray of shape (N, 3, 3)
        self.attachment_end = attachment_end    # np.ndarray of shape (N,)
    


class Trajectory:
    """
    ## @brief Data structure to hold trajectory information
    ##
    ## This class encapsulates the trajectory data for a manipulator, including
    ## joint positions, velocities, accelerations, torques, and base wrench forces
    ## at discrete time points.
    ##
    ## @param time np.ndarray: Array of time points.
    ## @param q np.ndarray: Joint positions at each time point.
    ## @param qdot np.ndarray: Joint velocities at each time point.
    ## @param qdotdot np.ndarray: Joint accelerations at each time point.
    ## @param tau np.ndarray: Joint torques at each time point.
    ## @param base_wrench np.ndarray: Base wrench forces at each time point.
    ## @param base_tf np.ndarray: Base transformation matrix.
    ## @param in_collision np.ndarray: Collision status at each time point.
    """
    def __init__(self, time, q, base_tf, attachment_end):
        self.time = time                  # np.ndarray of shape (N,)
        self.q = q                        # np.ndarray of shape (N, n)
        self.base_tf = base_tf            # np.ndarray of shape (3, 3)
        
        N, n = self.q.shape

        # Initialize dependent quantities as arrays of the correct shape
        self.qdot = np.zeros((N-1, n), dtype=np.float64)      # shape (N-1, n)
        self.qdotdot = np.zeros((N-2, n), dtype=np.float64)   # shape (N-2, n)
        self.tau = np.zeros((N, n), dtype=np.float64)         # shape (N, n)
        self.base_wrench = np.zeros((N, 3), dtype=np.float64) # shape (N, 3)
        self.in_collision = np.zeros((N,), dtype=bool)  # shape (N,)
        
def evaluate_path(path: Path, manip: PlanarManipulator):
    pass


# calculate outputs based on trajectory
def evaluate_trajectory(traj: Trajectory, manip: PlanarManipulator, obj=None):
    ## for all timepoints

    # calculate dt
    dt = calc_array_diff_array(traj.time)
    
    # calculate qdot
    dq = calc_array_diff_array(traj.q)
    print(f"dq: {dq} | dt: {dt} | traj.q: {traj.q}")
    traj.qdot = dq / dt[:, np.newaxis]

    # calculate qdotdot
    dqdot = calc_array_diff_array(traj.qdot)
    dqdt = get_array_midpoints(dt)
    traj.qdotdot = dqdot / dqdt[:, np.newaxis]
    
    for i in range(len(traj.time)):
        
        # calculate torque # static for now
        traj.tau[i] = calculate_torque(manip, traj.base_tf, traj.q[i])

        # calculate base wrench force
        traj.base_wrench[i] = calculate_base_wrench_force(manip=manip, base_tf=traj.base_tf, q=traj.q[i], tau=traj.tau[i])
    
        # check if in collision
        traj.in_collision[i] = manip.in_collision(traj.q[i], objs=obj,base_transform=traj.base_tf)
         
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

    if len(q) != manip.n:
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
        for j in range(i+1, manip.n+1):
            
            # Calculate Gravity force of link j in frame i
            tau_ij = calculate_torque_effect(manip.link_masses[j],joint_tfs[i][0:2,2],joint_tfs[j][0:2,2],g)
            
            tau[i] += tau_ij
                
        
    return tau

def calculate_torque_effect(m_j, pos_i, pos_j, g= -9.81):
    
    F = np.array([0.0, m_j * g])     # gravity force
    r = pos_j - pos_i               # moment arm
    return r[0]*F[1] - r[1]*F[0]     # 2D cross product

    G = np.array([0, m_j * g]) 
    V = pos_j - pos_i
    
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
    
    return G_rotated[1]  * d

# calculate base wrench force
def calculate_base_wrench_force(manip: PlanarManipulator, base_tf, q, qdot=None, qdotdot=None, tau = None, tfs = None):
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
    
    assert len(q) == manip.n, f"Expected {manip.n} joint angles, got {len(q)}"
    assert len(manip.link_masses) == manip.n + 1, "Link masses length mismatch"

    if tau is None:
        tau = calculate_torque(manip, base_tf, q, qdot, qdotdot)
    else:
        assert len(tau) == manip.n, f"Expected {manip.n} joint torques, got {len(tau)}"

    if tfs is None:
        tfs = manip.forward_kinematics(base_tf, q)
    else:
        assert len(tfs) == manip.n + 1, (
            f"Expected {manip.n + 1} forward kinematics transforms, got {len(tfs)}"
        )
    
    g = np.array([0, -9.81])
        
    # Wrench 
    W = np.zeros(3)
    
    # Rotation from world to base frame    
    R_world_to_base = base_tf[0:2,0:2]
    
    # for each joint
    for i in range(0, manip.n+1):
        ## Calculate the force at each joint
        f_i =  manip.link_masses[i] * g
        
        ## Rotate force into base frame
        f_i = R_world_to_base.T @ f_i
        
        ## Calculate torque at base due to this force
        base_tau = calculate_torque_effect(manip.link_masses[i],base_tf[0:2,2],tfs[i][0:2,2],g[1])
        tau_i = tau[i] if i<manip.n else 0.0
        base_tau = tau_i + base_tau
        
        ## Form 2D wrench vector
        W[0] += f_i[0]
        W[1] += f_i[1]
        W[2] += base_tau
        
    
        
    return W


