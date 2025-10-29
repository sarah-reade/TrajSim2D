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


# Initialise environment

## Generate random border

def generate_random_border(border_size=1, smoothness=1, num_points=200):
    """
    @brief Generate a random circular border with smooth, hill-like bumps.
    @param border_size Approximate diameter; border centered at border_size/2.
    @param smoothness Float [0,1], 1 = perfect circle, 0 = maximum bumps.
    @param num_points Number of points along the border.
    @return Nx2 np.ndarray of (x, y) points defining the border.
    """
    center = border_size / 2
    radius = border_size / 2
    
    angles = np.linspace(0, 2*np.pi, num_points, endpoint=False)
    
    # Random positive bumps
    np.random.seed()
    max_bump = radius * (1 - smoothness)
    bumps = np.abs(np.random.randn(num_points)) * max_bump
    
    # Apply Gaussian smoothing to make hills rounder
    sigma = num_points / 20  # adjust for hill roundness
    bumps = gaussian_filter1d(bumps, sigma)
    
    # Final radius with smooth bumps
    r = radius + bumps
    
    # Convert polar to Cartesian, centered
    x = center + r * np.cos(angles)
    y = center + r * np.sin(angles)
    
    return np.column_stack((x, y))
## 

import matplotlib.pyplot as plt

for s in [1.0, 0.75, 0.5, 0.0]:
    border = generate_random_border(border_size=1, smoothness=s)
    plt.plot(border[:,0], border[:,1], label=f'smoothness={s}')
plt.gca().set_aspect('equal')
plt.legend()
plt.show()