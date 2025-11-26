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
    collision_flag, overlaps = get_AABB_Overlap(shape_1,shape_2,inflation)
    return collision_flag, overlaps


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

    @param shapes_1 List of shapes representing the first set of shapes to check for collisions.
    @param shapes_2 List of shapes representing the second set of shapes to check against shapes_1.
    @param max_distance Float representing the maximum distance to obtain a measurement of how far objects are away from each other.
    @return bool True if any collision is detected, False otherwise.
    @return collision_shapes_id List of tuples (shape_1, shape_2, distance) representing which shapes are colliding and the distance between them.
    """

    # Create a list of potentially intersecting shapes and their potentially intersecting areas using bounding boxes
    any_collisions_flag, pot_collision_shapes = detect_any_collisions_AABB(shapes_1,shapes_2,inflation=0.1)

    if not any_collisions_flag:
        return False,[]
    
    # For all potentially intersecting shapes check for more in detail collisions:
    #any_collisions_flag, collision_distances = detect_any_collisions_DISTANCE(pot_collision_shapes)


    collision_distances = []
    return any_collisions_flag, collision_distances

def detect_any_collisions_AABB(shapes_1,shapes_2,inflation=0.1):
    """
    @brief Checks for collisions between two sets of shapes using AABB (Axis-Aligned Bounding Box).

    @param shapes_1 List of shapes to check for collisions.
    @param shapes_2 List of shapes to check against shapes_1.
    @param inflation Optional float. Margin to inflate the bounding boxes when checking for collisions. Default is 0.1.

    @return bool True if no collisions are detected, False otherwise.
    @return list potential_collisions List of tuples (shape_1, shape_2, intersecting_area) for all detected collisions.
    """

    potential_collisions = []

    for i, shape_1 in enumerate(shapes_1):
        for shape_2 in shapes_1[i+1:]+shapes_2:
            if shape_1 is shape_2:
                continue

            collision, intesecting_area = detect_collision_AABB(shape_1,shape_2,inflation)
            if collision:
                potential_collisions.append((shape_1,shape_2,intesecting_area))
            
    return bool(potential_collisions), potential_collisions
            
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



def get_AABB(shape, inflation=0.0):
    """
    @brief returns min and max x and y coordinates for the shape

    @param shape Rectangle, Circle, or np.ndarray shape to get bounding values for
    @param inflation Float amount to inflate the min and max values
    @return Tuple (minx, miny, maxx, maxy)
    """

    # LOGIC
    if isinstance(shape,Circle):
        minx,miny,maxx,maxy = get_AABB_Circle(shape)
    elif isinstance(shape,Rectangle):
        minx,miny,maxx,maxy = get_AABB_Rectangle(shape)
    elif isinstance(shape,np.ndarray):
        minx,miny,maxx,maxy = get_AABB_Array(shape)
    else:
        minx,miny,maxx,maxy = 0.0,0.0,0.0,0.0

    return (minx-inflation,miny-inflation,maxx+inflation,maxy+inflation)

def get_AABB_Circle(circle):
    """
    @brief Returns the axis-aligned bounding box (AABB) of a matplotlib Circle.

    @param circle Circle object.
    @return (minx, miny, maxx, maxy). Returns zeros if not a Circle.
    """

    if not isinstance(circle,Circle):
        return 0.0,0.0,0.0,0.0

    cx, cy = circle.center
    r = circle.radius

    minx = cx - r
    maxx = cx + r
    miny = cy - r
    maxy = cy + r
    return minx, miny, maxx, maxy

def get_AABB_Rectangle(rectangle):
    """
    @brief Returns the axis-aligned bounding box (AABB) of a matplotlib Rectangle.

    Supports rotated rectangles by transforming corner points.

    @param rectangle Rectangle object.
    @return (minx, miny, maxx, maxy). Returns zeros if not a Rectangle.
    """
    if not isinstance(rectangle,Rectangle):
        return 0.0,0.0,0.0,0.0
    
    # Extract rectangle properties
    x = rectangle.get_x()
    y = rectangle.get_y()
    w = rectangle.get_width()
    h = rectangle.get_height()
    angle = rectangle.angle  # degrees, CCW, about (x, y)

    # Unrotated corner points
    corners = np.array([
        [x,     y    ],  # bottom-left (rotation anchor)
        [x+w,   y    ],  # bottom-right
        [x+w,   y+h  ],  # top-right
        [x,     y+h  ]   # top-left
    ])

    # If not rotated, fast path
    if angle == 0:
        xs = corners[:,0]
        ys = corners[:,1]
        return xs.min(), ys.min(), xs.max(), ys.max()

    # Rotation matrix
    theta = np.deg2rad(angle)
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

    # Rotate around (x, y)
    rotated = (R @ (corners - np.array([x, y])).T).T + np.array([x, y])

    xs = rotated[:,0]
    ys = rotated[:,1]

    return xs.min(), ys.min(), xs.max(), ys.max()

def get_AABB_Array(array):
    """
    @brief Returns the axis-aligned bounding box (AABB) of a 2D point array.

    @param array Nx2 numpy array of (x, y) points.
    @return (minx, miny, maxx, maxy). Returns zeros if not an array.
    """
    if not isinstance(array,np.ndarray):
        return 0.0,0.0,0.0,0.0

    xs = array[:,0]
    ys = array[:,1]

    minx = xs.min()
    maxx = xs.max()
    miny = ys.min()
    maxy = ys.max()

    return minx, miny, maxx, maxy

def get_AABB_Overlap(shape_1,shape_2, inflation=0.0):
    """
    @brief determines the overlap between two shapes
    
    @param shape_1 Rectangle, Circle, np.ndarray, or Tuple  (minx, miny, maxx, maxy)
    @param shape_2 Rectangle, Circle, np.ndarray, or Tuple  (minx, miny, maxx, maxy)
    @return bool if there is any overlap
    @return Tuple (minx, miny, maxx, maxy) overlap bounds
    """
    if not isinstance(shape_1,tuple):
        shape_1 = get_AABB(shape_1,inflation)

    if not isinstance(shape_2,tuple):
        shape_2 = get_AABB(shape_2,inflation)
    
    overlap = False
    overlap_bounds = [0.0,0.0,0.0,0.0]

    minx1, miny1, maxx1, maxy1 = np.array(shape_1,dtype=float)
    minx2, miny2, maxx2, maxy2 = np.array(shape_2,dtype=float)

    overlap_x = maxx1 > minx2 and maxx2 > minx1
    overlap_y = maxy1 > miny2 and maxy2 > miny1
    overlap = overlap_x and overlap_y

    if not overlap:
        return False, (0.0, 0.0, 0.0, 0.0)

    overlap_bounds = (
        max(minx1, minx2),
        max(miny1, miny2),
        min(maxx1, maxx2),
        min(maxy1, maxy2)
    )

    return True, overlap_bounds