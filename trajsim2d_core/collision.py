###############################################################################
## @file collision.py
## @brief Collision checking functions for TrajSim2D.
##
## This file is part of the TrajSim2D project, a 2D planar manipulator simulator
## for trajectory planning, collision testing, and environment visualization.
## 
## Author: Sarah Reade
## Email: 28378329@students.lincoln.ac.uk
## Date: 2025-10-23
## Version: 0.0.1
##
## License: MIT
##
## Usage:
## >>> import trajsim2d_core.collision as collision
###############################################################################

# Imports
import numpy as np

# Define Circle
class Circle:
    def __init__(self, radius: float, transform: np.ndarray):
        self.radius = radius
        self.transform = transform  # 2D transform 

# Define Rectangle
class Rectangle:
    def __init__(self, width: float, length: float, transform=np.ndarray):
        self.width = width
        self.length = length
        self.transform = transform  # 2D transform 

## Detect Collision
def detect_collision(shape_1,shape_2):
    """
    @brief Check for collisions between two shapes
    @param shape_1 is a shape to check for collisions against shape_2
    @param shape_2 is a shape to check for collisions against shape_1
    @return bool: if collision is detected
    """
    # if either shape is np.ndarray decompose into convex shapes

    # 

    # detect between a rectangle v rectangle

    # detect between a rectangle v circle

    # detect between a rectangle v np.ndarry

    return False

def detect_shapes_bounded(boundary,shapes):
    """
    @brief checks that the shapes are contained in the boundary 
    @param boundary is a shape area to check the shapes are contained within
    @param shapes is a list of shapes to check they are contained within the boundary
    @return bool: if shapes_1 are in bounds
    """
    if boundary is None:
        return True
    
    if shapes is None:
        return True
    


    return False

def detect_any_collisions(shapes_1,shapes_2):
    """
    @brief Iterates through all shapes to check for collisions between them
    @param shapes_1 is a list of shapes to check for collisions of inclusive
    @param shapes_2 is a list of shapes to check for collisions of against shapes_1
    @return bool: if collision is detected
    """

    for shape_1 in shapes_1:
        for shape_2 in shapes_1+shapes_2:
            if shape_1 is shape_2:
                continue
            if detect_collision(shape_1,shape_2):
                return True
    return False

def detect_any_collisions_bounded(boundary,shapes_1,shapes_2):
    """
    @brief checks that the shapes_1 are within the boundary and theer are no collisions
    internally between shapes_1 and between shapes_1 and shapes_2
    @param boundary is a shape area to check the shapes_1 are contained within
    @param shapes_1 is a list of shapes to check for collisions of inclusive
    @param shapes_2 is a list of shapes to check for collisions of against shapes_1
    @return bool: if collision is detected or shapes_1 out of bounds
    """

    return detect_any_collisions(shapes_1,shapes_2) and not detect_shapes_bounded(boundary,shapes_1)


