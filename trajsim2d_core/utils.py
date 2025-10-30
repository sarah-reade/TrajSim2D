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
def generate_random_number(min_val, max_val):
    """
    @brief Generate a random floating-point number between min_val and max_val.
    @param min_val Minimum value (inclusive).
    @param max_val Maximum value (exclusive).
    @return Random float in [min_val, max_val).
    """
    rng = np.random.default_rng()  # automatically random seed
    output = rng.uniform(min_val, max_val)
    #print("min-max: ",min_val, "-",max_val)
    return output

## Generate random int
def generate_random_int(min_val, max_val):
    """
    @brief Generate a random floating-point number between min_val and max_val.
    @param min_val Minimum value (inclusive).
    @param max_val Maximum value (exclusive).
    @return Random float in [min_val, max_val).
    """
    return int(generate_random_number(min_val, max_val))

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