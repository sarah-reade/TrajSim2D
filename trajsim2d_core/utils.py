###############################################################################
## @file utils.py
## @brief Utility functions for TrajSim2D.
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
## >>> import trajsim2d_core.utils as utils
###############################################################################

# Imports
import numpy as np

## Generate random number
def generate_random_number(min_val, max_val, size=None):
    """
    @brief Generate one or more random floating-point numbers between min_val and max_val.
    @param min_val Minimum value (inclusive).
    @param max_val Maximum value (exclusive).
    @param size Number of random values to generate. If None, returns a single float.
    @return Single float if size is None, otherwise np.ndarray of random floats.
    """
    rng = np.random.default_rng()  # auto-seeded with system entropy
    output = rng.uniform(min_val, max_val, size=size)
    return output

## Generate random int
def generate_random_int(min_val, max_val):
    """
    @brief Generate a random floating-point number between min_val and max_val.
    @param min_val Minimum value (inclusive).
    @param max_val Maximum value (exclusive).
    @return Random float in [min_val, max_val).
    """
    val = generate_random_number(min_val, max_val)
    return int(np.rint(val).astype(int))

## Generate random array of ints
def generate_random_int_array(low, high, size=None):
    """
    @brief Generate a random floating-point number between min_val and max_val.
    @param min_val Minimum value (inclusive).
    @param max_val Maximum value (exclusive).
    @param size Array size.
    @return Random float in [min_val, max_val).
    """
    rng = np.random.default_rng()
    return rng.integers(low, high, size=size)

## Generate random arm parameters

## Generate random environment parameters

## Generate direct trajectory

## Save outputs 

## Calculate Array Diffs
def calc_array_diff(array):
    """
    @brief Calculate differences between consecutive elements in an array or list.
    @param array List or NumPy array of floats.
    @return List or NumPy array of float differences, matching the input type.
    """
    data = np.array(array, dtype=float)  # ensure it's a NumPy array of floats
    diff = np.diff(data)
    # Return same type as input
    return diff.tolist() if isinstance(array, list) else diff


def get_random_element(array):
    """
    @brief Select a random element from a 1D list or 1D np.ndarray.
    @param array 1D list or np.ndarray.
    @return Random element from the array/list.
    """
    if isinstance(array, np.ndarray):
        length = array.size
    else:
        length = len(array)
    idx = generate_random_int(0, length - 1)
    return array[idx]


def get_random_coordinate(array2d):
    """
    @brief Select a random row from a 2D np.ndarray.
    @param array2d NxM np.ndarray, each row is a coordinate.
    @return 1D np.ndarray of length M representing a randomly chosen coordinate.
    """
    import numpy as np

    if isinstance(array2d, list):
        array2d = np.array(array2d)

    if array2d.ndim != 2:
        raise ValueError(f"Expected a 2D array, got shape {array2d.shape}")

    idx = generate_random_int(0, array2d.shape[0] - 1)
    return array2d[idx], idx


def get_random_array_from_list(array_list):
    """
    @brief Select a random np.ndarray from a list of arrays.
    @param array_list List of np.ndarray objects.
    @return Randomly selected np.ndarray from the list.
    """
    if not array_list:
        raise ValueError("Input list is empty")
    idx = generate_random_int(0, len(array_list) - 1)
    return array_list[idx]

def tangent_angle_at_point(array, idx):
    """
    @brief Compute a tangent angle at a point.
    @param array Nx2 np.ndarray of coordinates (assumed ordered along curve).
    @param idx Index of the point of interest.
    @return angle Angle in radians measured clockwise from the positive y-axis.
    """
    
    N = array.shape[0]

    # Wrap indices for closed curve
    prev_idx = (idx - 1) % N
    next_idx = (idx + 1) % N

    p_prev = array[prev_idx]
    p_curr = array[idx]
    p_next = array[next_idx]
    #print("p_prev:",p_prev)
    #print("p_curr:",p_curr)
    #print("p_next:",p_next)

    # Vector: prev->next
    v = p_next - p_prev

    # Angle y/x
    angle = np.arctan2(v[1], v[0])
    #print("angle:",angle)
    return angle


def normal_angle_at_point(array, idx):
    """
    @brief Compute the outward normal angle at a point on a closed 2D curve.
    @param array Nx2 np.ndarray of coordinates (assumed ordered along a closed curve).
    @param idx Index of the point of interest.
    @return normal_angle Angle in radians (clockwise from +Y) of the outward normal.
    @details
    The outward normal is computed by rotating the tangent by ±90° depending on
    the contour winding direction:
      - For CCW contours, the outward normal is tangent - π/2.
      - For CW contours, the outward normal is tangent + π/2.
    """
    tangent = tangent_angle_at_point(array, idx)

    # Determine winding (shoelace area)
    area = 0.5 * np.sum(array[:-1, 0] * array[1:, 1] - array[1:, 0] * array[:-1, 1])

    if area > 0:  # CCW contour
        normal = tangent - np.pi / 2
    else:          # CW contour
        normal = tangent + np.pi / 2

    return normal % (2 * np.pi)


def make_transform_2d(tx=0.0, ty=0.0, theta=0.0):
    """
    @brief Create a 2D transformation matrix combining rotation and translation.
    @details The transform first applies rotation (about the origin),
             then translation, in homogeneous coordinates.
    @param tx Translation along x-axis.
    @param ty Translation along y-axis.
    @param theta Rotation angle in radians (counter-clockwise).
    @return 3x3 np.ndarray representing the combined transform.
    """
    c, s = np.cos(theta), np.sin(theta)
    transform = np.array([
        [c, -s, tx],
        [s,  c, ty],
        [0,  0, 1]
    ])
    return transform

def getRectAngle(tf):
    """
    @brief Compute the rotation angle of a rectangle from a 3x3 transform.
    
    @param tf 3x3 homogeneous transform matrix (world → rectangle).
    @return Rotation angle in degrees.
    """
    return np.degrees(np.arctan2(tf[1, 0], tf[0, 0]))


def getRectAnchor(tf, w, h):
    """
    @brief Compute the lower-left corner (anchor) of a rectangle for matplotlib.
    
    @param tf 3x3 homogeneous transform (world → rectangle).
    @param w Width of the rectangle.
    @param h Height of the rectangle.
    @return (x, y) coordinates of the lower-left corner.
    """
    xc, yc = tf[0, 2], tf[1, 2]
    theta = np.arctan2(tf[1, 0], tf[0, 0])
    c, s = np.cos(theta), np.sin(theta)
    x0 = xc - (w/2)*c + (h/2)*s
    y0 = yc - (w/2)*s - (h/2)*c
    return (x0, y0)


def getRectRotPoint(tf, w, h):
    """
    @brief Get the rotation point of a rectangle for matplotlib.
    @details In matplotlib, rotation occurs about the lower-left corner,
             so this is the same as the anchor.
    
    @param tf 3x3 homogeneous transform (world → rectangle).
    @param w Width of the rectangle.
    @param h Height of the rectangle.
    @return (x, y) coordinates of the rotation point.
    """
    return getRectAnchor(tf, w, h)