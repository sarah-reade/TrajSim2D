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