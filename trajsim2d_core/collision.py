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
from matplotlib.patches import Polygon, Rectangle, Circle


## Detect Collision
def detect_collision_AABB(shape_1,shape_2,inflation=0.1):
    """
    @brief Check for collisions between two shapes using bounding boxes/circles
    @param shape_1 is a shape to check for collisions against shape_2
    @param shape_2 is a shape to check for collisions against shape_1
    @return bool: if collision is detected
    """


    # detect between a rectangle v rectangle

    # detect between a rectangle v circle

    # detect between a rectangle v np.ndarry

    return False, intesecting_area


def detect_collision_DISTANCE(shape_1,shape_2,intersecting_area):
    """
    @brief Check for collisions between two shapes using a complex method and measuring distance between
    @param shape_1 is a shape to check for collisions against shape_2
    @param shape_2 is a shape to check for collisions against shape_1
    @return bool: if collision is detected
    """


    # detect between a rectangle v rectangle

    # detect between a rectangle v circle

    # detect between a rectangle v np.ndarry

    return False, distance


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

def detect_any_collisions(shapes_1,shapes_2,max_distance=0.1):
    """
    @brief Iterates through all shapes and checks for collisions between them.

    @param shapes_1 List of tuples (shape, shape_id) representing the first set of shapes to check for collisions.
    @param shapes_2 List of tuples (shape, shape_id) representing the second set of shapes to check against shapes_1.
    @param max_distance Float representing the maximum distance to obtain a measurement of how far objects are away from each other.
    @return bool True if any collision is detected, False otherwise.
    @return collision_shapes_id List of tuples (shape_1_id, shape_2_id, distance) representing which shapes are colliding and the distance between them.
    """

    # Create a list of potentially intersecting shapes and their potentially intersecting areas using bounding boxes
    any_collisions_flag, pot_collision_shapes = detect_any_collisions_AABB(shapes_1,shapes_2,inflation=0.1)

    if not any_collisions_flag:
        return False,[]
    
    # For all potentially intersecting shapes check for more in detail collisions:
    any_collisions_flag, collision_distances = detect_any_collisions_DISTANCE(pot_collision_shapes)


    
    return any_collisions_flag, collision_distances

def detect_any_collisions_AABB(shapes_1,shapes_2,inflation=0.1):
    """
    @brief Checks for collisions between two sets of shapes using AABB (Axis-Aligned Bounding Box).

    @param shapes_1 List of shapes (with associated IDs or references) to check for collisions.
    @param shapes_2 List of shapes (with associated IDs or references) to check against shapes_1.
    @param inflation Optional float. Margin to inflate the bounding boxes when checking for collisions. Default is 0.1.

    @return bool True if no collisions are detected, False otherwise.
    @return list potential_collisions List of tuples (shape_1, shape_2, intersecting_area) for all detected collisions.
    """

    potential_collisions = []

    for shape_1 in shapes_1:
        for shape_2 in shapes_1+shapes_2:
            if shape_1 is shape_2:
                continue

            collision, intesecting_area = detect_collision_AABB(shape_1,shape_2,inflation)
            if collision:
                potential_collisions.append((shape_1,shape_2,intesecting_area))
            
    return not potential_collisions, potential_collisions
            
def detect_any_collisions_DISTANCE(shapes):
    """
    @brief Checks for collisions or distances between shape pairs using a distance-based algorithm (placeholder).

    @param shapes List of tuples (shape_1, shape_2, intersecting_area) representing potential collisions.

    @return bool True if any collision is detected, False otherwise.
    @return list distances List of tuples (shape_1, shape_2, distance) giving the computed distance or overlap between each shape pair.
    """

    collision_flag = False
    distances = []

    for shape_1, shape_2, intersecting_area in shapes:
            
        collision, distance = detect_collision_DISTANCE(shape_1,shape_2,intersecting_area)
        if collision:
            collision_flag = True
        distances.append((shape_1,shape_2,distance))
            
    return collision_flag, distances

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



