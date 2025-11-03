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

# Initialise visualisation 
def initialise_visualisation(border=None,objects=None,base_transform=None,link_width=0.01,link_lengths=0.5,joint_config_1=None,joint_config_2=None):
    """
    @brief Initialise a 2D visualisation with a border.
    @param border Nx2 np.ndarray defining the boundary.
    @param objects Placeholder for objects (ignored for now).
    @param base_transform Placeholder (ignored for now).
    @param line_width Width of the border line.
    @param link_lengths Placeholder (ignored for now).
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
    
    ## Add Arm


    return canvas, border_id, object_ids

# Initialise Arm



# Sync visualise trajectory

# Async visualise trajectory

# Visualise outputs 

# Wait for visualisation of trajectory to be over