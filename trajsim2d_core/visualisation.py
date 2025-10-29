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
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

# Initialise visualisation 
def initialise_visualisation(border,objects=None,base_transform=None,link_width=0.01,link_lengths=0.5,joint_config_1=None,joint_config_2=None):
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
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    
    initialise_figure(ax, border)
    
    # Set axis limits slightly beyond border
    margin = 0.1
    x_min, y_min = border.min(axis=0) - margin
    x_max, y_max = border.max(axis=0) + margin
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    
    plt.ion()
    plt.show()
    
    return fig, ax

## initialise graph
def initialise_figure(ax, border, line_width = 0.01):
    """
    @brief Draws a closed polygon for the border.
    @param ax Matplotlib axis.
    @param border Nx2 np.ndarray defining the boundary.
    @param line_width Width of the border line.
    """
    polygon = Polygon(border, closed=True, fill=False, edgecolor='black', linewidth=line_width)
    ax.add_patch(polygon)

    margin = 0.1
    ax.set_xlim(border[:,0].min() - margin, border[:,0].max() + margin)
    ax.set_ylim(border[:,1].min() - margin, border[:,1].max() + margin)
    ax.set_aspect('equal')

## initialise objects

# Sync visualise trajectory

# Async visualise trajectory

# Visualise outputs 

# Wait for visualisation of trajectory to be over