###############################################################################
## @file test_visualisation.py
## @brief Test Visualisation functions for TrajSim2D.
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
## >>> pytest
###############################################################################

import unittest
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

from trajsim2d_core.environment import generate_random_border
from trajsim2d_core.visualisation import initialise_visualisation



class TestVisualisation(unittest.TestCase):
    def test_initialise_visualisation_border_user_confirm(self):
        # Generate a random bumpy border for testing
        border = generate_random_border(border_size=1, smoothness=2)
        print(border.shape)  # Should be (N, 2)
        print(border[:5])    # Sample first few points
        # Initialise the visualisation
        fig, ax = initialise_visualisation(border)
        
        # Draw and show the figure
        plt.draw()
        plt.pause(0.1)  # ensure it renders
        
        # Ask the user for confirmation
        user_input = input("Do you see the border correctly? (y/n): ").strip().lower()
        plt.close(fig)
        
        # Pass/fail based on user input
        self.assertIn(user_input, ['y', 'yes'], msg="User indicated the border is not correct.")

if __name__ == "__main__":
    unittest.main()