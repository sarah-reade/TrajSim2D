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
from trajsim2d_core.utils import generate_random_arm, make_transform_2d, generate_random_number
from trajsim2d_core.environment import generate_random_edge_point
import numpy as np

# Initialise visualisation 
def initialise_visualisation(border=None,objects=None,base_transform=None,arm=None,joint_config_1=None,joint_config_2=None):
    """
    @brief Initialise a 2D visualisation with a border.
    @param border Nx2 np.ndarray defining the boundary.
    @param objects Placeholder for objects (ignored for now).
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
        border_id = canvas.add_array(border)

    ## Add objects
    object_ids = []
    if objects is not None:
        for object in objects:
            id = canvas.add_array(object)
            object_ids.append(id)
    
    ## Add Arm (if one of border and objects exist)
    if arm is not None:
        visualise_arm(canvas,arm,border,objects,base_transform,joint_config_1,joint_config_2)


    return canvas, border_id, object_ids

# Initialise Arm
def visualise_arm(canvas,arm,border=None,objects=None,base_transform=None,joint_config_1=None,joint_config_2=None):
    """
    @ brief adds an arm to the canvas
    """
    ## generate the base_transform
    if base_transform is None:
        base_transform = make_transform_2d() 

        if border is not None or objects is not None:
            point, angle = generate_random_edge_point(border,objects,)
            base_transform = make_transform_2d(point[0],point[1],angle)

    ## generate arm is not defined
    

    ## Generate a valid joint config
    config_1 = generate_random_number(-np.pi,np.pi,len(link_lengths))
    
    ## Iterate to find one that does not collide



# Sync visualise trajectory

# Async visualise trajectory

# Visualise outputs 

# Wait for visualisation of trajectory to be over