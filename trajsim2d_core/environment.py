###############################################################################
## @file environment.py
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
from scipy.ndimage import gaussian_filter1d  # smoother hills
from trajsim2d_core.utils import generate_random_number,generate_random_int,generate_random_int_array

# Initialise environment
BORDER_SIZE = 5
BORDER_CENTRE = BORDER_SIZE/2

## Generate random border
def generate_random_border(border_size=BORDER_SIZE, smoothness=1, num_points=200):
    """
    @brief Generate a smooth, random circular border with periodic bumps.
    """
    
    BORDER_SIZE = border_size
    BORDER_CENTRE = border_size/2
    return generate_random_circle(border_size,smoothness,num_points)


def generate_random_objects(num_points=60,object_size=None,object_max=None,smoothness=None,num_objects=None):
    """
    @brief Generate a smooth, random circular object with periodic bumps.
    """
    objects = []
    
    if object_size != None:
        object_max = object_size
    
        
    elif object_max == None:
        object_max = generate_random_number(0,BORDER_SIZE/4)
    
    
    #print("object_size: ",object_size)
    #print("object_max: ",object_max)    

    if num_objects == None:
        num_objects = generate_random_int(1,(BORDER_SIZE/object_max))
    
    #print("num_objects: ",num_objects)

    for i in range(0,num_objects):
        if smoothness == None:
            smoothness = generate_random_number(0,1)

        #print("smoothness: ",smoothness)

        if object_size == None:
            object_i_size = generate_random_number(0.1,object_max)
        else:
            object_i_size = object_size

        #print("object_i_size: ",object_i_size)
        centre = [generate_random_number(0.1,BORDER_SIZE),generate_random_number(0.1,BORDER_SIZE)]

        objects.append(generate_random_circle(circle_size=object_i_size,smoothness=smoothness,num_points=num_points,circle_centre=centre))

    return objects

    

## Generate a bumby circle 
def generate_random_circle(circle_size=1, smoothness=1, num_points=200,circle_centre=None):
    """
    @brief Generate a smooth, random circle with periodic bumps.
    """
    if circle_centre == None:
        circle_centre = [circle_size / 2,circle_size / 2]

    radius = circle_size / 2
    
    angles = np.linspace(0, 2*np.pi, num_points, endpoint=False)
    
    # Random bumps

    max_bump = radius * (1 - smoothness)
    bumps = np.abs(generate_random_int_array(10,num_points,num_points)) * max_bump
    
    # Circular Gaussian smoothing (wrap mode)
    sigma = num_points / 60
    #print("bumps: ",bumps)
    #print("sigma: ",sigma)
    #print("num_points",num_points)
    bumps = gaussian_filter1d(bumps, sigma, mode='wrap')
    
    # Final radius
    r = radius + bumps
    
    # Convert polar to Cartesian
    x = circle_centre[0] + r * np.cos(angles)
    y = circle_centre[1] + r * np.sin(angles)
    
    return np.column_stack((x, y))
