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

from trajsim2d_core.environment import generate_random_border, generate_random_objects
from trajsim2d_core.visualisation import initialise_visualisation



class TestVisualisation(unittest.TestCase):
    def test_initialise_visualisation_border_user_confirm(self):
        # Generate a random bumpy border for testing
        border = generate_random_border(border_size=5, smoothness=0.5)

        # Initialise the visualisation
        canvas, border_id, object_ids  = initialise_visualisation(border)
        
        # Ask the user for confirmation
        user_input = input("Do you see the border correctly? (y/n): ").strip().lower()
        
        # Pass/fail based on user input
        self.assertIn(user_input, ['y', 'yes'], msg="User indicated the border is not correct.")

    def test_initialise_visualisation_objects_user_confirm(self):
        # Generate a random bumpy border for testing
        objects = generate_random_objects(object_size=0.5,num_objects=5,smoothness=0.5)
        # Initialise the visualisation
        canvas, border_id, object_ids  = initialise_visualisation(objects=objects)
        
        # Ask the user for confirmation
        user_input = input("Do you see the objects correctly? (y/n): ").strip().lower()
        
        # Pass/fail based on user input
        self.assertIn(user_input, ['y', 'yes'], msg="User indicated the border is not correct.")

    ##def test_initialise_arm(self):


if __name__ == "__main__":
    unittest.main()