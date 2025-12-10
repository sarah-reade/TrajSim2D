###############################################################################
## @file visualisation.py
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
from trajsim2d_core.geometry_canvas import GeometryCanvas
from trajsim2d_core.utils import make_transform_2d, generate_random_number
from trajsim2d_core.environment import generate_random_edge_point
from trajsim2d_core.twodmanip import PlanarManipulator
import numpy as np

# Initialise visualisation 
def initialise_visualisation(border=None,objs=None,base_transform=None,arm=None,joint_config_1=None,joint_config_2=None):
    """
    @brief Initialise a 2D visualisation with a border.
    @param border Nx2 np.ndarray defining the boundary.
    @param objs Placeholder for objects (ignored for now).
    @param base_transform Placeholder (ignored for now).
    @param arm 2dmanip Object.
    @param joint_config_1 Placeholder (ignored for now).
    @param joint_config_2 Placeholder (ignored for now).
    @return Figure and axis handles.
    """
    canvas = GeometryCanvas()
    ## Add border
    border_id = 0
    if border is not None:
        border_id = visualise_object(canvas,border,'olive')

    ## Add objects
    object_ids = []
    if objs is not None:
        object_ids = visualise_object(canvas,objs,'black')
    
    ## Add Arm (if one of border and objects exist)
    arm_ids = []
    if arm is not None:
        arm_ids, base_transform = visualise_arm(canvas,arm,border,objs,base_transform,joint_config_1,joint_config_2)
        base_id = canvas.add_tf(base_transform)

    return canvas, base_transform, border_id, object_ids, arm_ids

# Initialise Arm
def visualise_arm(canvas,arm=PlanarManipulator(),border=None,objs=None,base_transform=None,joint_config_1=None,joint_config_2=None):
    """
    @ brief adds an arm to the canvas
    """
    ## generate the base_transform
    if base_transform is None:
        base_transform = make_transform_2d() 

        if border is not None or objs is not None:
            point, angle = generate_random_edge_point(border,objs)
            base_transform = make_transform_2d(point[0],point[1],angle)


    ## Generate a valid joint config
    if joint_config_1 is None:
        joint_config_1 = arm.generate_random_config(border,objs)
    
    if joint_config_2 is None:
        joint_config_2 = arm.generate_random_config(border,objs)
    

    ## visualise on canvas
    arm_geometry_1 = arm.make_arm_geometry(joint_config_1,base_transform)
    arm_geometry_2 = arm.make_arm_geometry(joint_config_2,base_transform)

    return visualise_object(canvas,arm_geometry_1,'green') + visualise_object(canvas,arm_geometry_2,'yellow'),base_transform

def visualise_object(canvas,obj,color='red', alpha=0.5):
    return canvas.add_shape(obj,color,alpha)


# Sync visualise trajectory

# Async visualise trajectory

# Visualise outputs 

# Wait for visualisation of trajectory to be over