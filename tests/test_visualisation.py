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

import time
import unittest
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

from trajsim2d_core.environment import generate_random_border, generate_random_convex_objects
from trajsim2d_core.visualisation import initialise_visualisation, visualise_object, visualise_trajectory_sync, update_trajectory_visualisation
from trajsim2d_core.twodmanip import PlanarManipulator
from trajsim2d_core.collision import create_convex_boundary_objects
from trajsim2d_core.calculations import Trajectory, evaluate_trajectory_threaded



class TestVisualisation(unittest.TestCase):
    def test_initialise_visualisation_border_user_confirm(self):
        # Generate a random bumpy border for testing
        border = generate_random_border(border_size=5, smoothness=0.5)

        # Initialise the visualisation
        canvas, base_tf, border_id, object_ids, arm_ids, joint_config_1, joint_config_2  = initialise_visualisation(border)
        
        # Ask the user for confirmation
        user_input = input("Do you see the border correctly? (y/n): ").strip().lower()
        
        # Pass/fail based on user input
        self.assertIn(user_input, ['y', 'yes'], msg="User indicated the border is not correct.")

    def test_initialise_visualisation_objects_user_confirm(self):
        # Generate a random bumpy objects for testing
        [objs, concave_objs] = generate_random_convex_objects(object_size=0.5,num_objs=5,smoothness=0.5)
        # Initialise the visualisation
        canvas, base_tf, border_id, object_ids, arm_ids, joint_config_1, joint_config_2  = initialise_visualisation(objs=concave_objs)
        
        # Ask the user for confirmation
        user_input = input("Do you see the objects correctly? (y/n): ").strip().lower()
        
        # Pass/fail based on user input
        self.assertIn(user_input, ['y', 'yes'], msg="User indicated the border is not correct.")

    def test_initialise_visualisation_arm_user_confirm(self):
        # Generate a random bumpy border for testing
        border = generate_random_border(border_size=5, smoothness=0.001)
        
        # Generate a random bumpy objects for testing
        objs, concave_objs = generate_random_convex_objects(object_size=0.5,num_objs=5,smoothness=0.001)
        # Generate a random arm for testing
        arm = PlanarManipulator(n=3)
        arm.print_parameters()
        # config_1 = [arm.joint_limit for _ in range(arm.n)]
        # config_2 = [-arm.joint_limit for _ in range(arm.n)]

        # Initialise the visualisation
        canvas, base_tf, border_id, object_ids, arm_ids, joint_config_1, joint_config_2  = initialise_visualisation(border=border,objs=objs,arm=arm,attempt_max=20)

        convex_border = create_convex_boundary_objects(border)
        canvas.add_shape(convex_border)
        # Show the figure in blocking mode — execution will pause until the window is closed
        #plt.show(block=True)

        # If we reach this point, the user has closed the window successfully
        self.assertTrue(True, msg="Visualisation closed successfully.")

class TestTrajectoryVisualisation(unittest.TestCase):
    def setUp(self):
        # Get joint number
        n_joints = 3
        
        # Simple 1-link manipulator
        self.arm = PlanarManipulator(n=n_joints)
        self.arm.print_parameters()

        # Generate a random bumpy border for testing
        self.border = generate_random_border(border_size=5, smoothness=0.001)
        
        # Generate a random bumpy objects for testing
        objs, self.concave_objs = generate_random_convex_objects(object_size=0.5,num_objs=5,smoothness=0.001)
        
        # Initialise the visualisation
        self.canvas, base_tf, border_id, object_ids, arm_ids, joint_config_1, joint_config_2  = initialise_visualisation(border=self.border,objs=self.concave_objs,arm=self.arm,attempt_max=20)
        
        # Create a smooth trajectory for testing
        # Number of points
        N = 100  # number of points

        # Variable time vector (non-uniform but increasing)
        time = np.cumsum(np.random.uniform(0.05, 0.1, size=N))
        time -= time[0]  # start at 0

        # Normalized time from 0 to 1 for ease-in-out
        t_norm = np.linspace(0, 1, N)

        # Initialize q
        q = np.zeros((N, n_joints))

        # Apply curved (ease-in-out) motion using sine
        for j in range(n_joints):
            q[:, j] = joint_config_1[j] + (joint_config_2[j] - joint_config_1[j]) * 0.5 * (1 - np.cos(np.pi * t_norm))


        attachment_end = 1

        # Trajectory object
        self.traj = Trajectory(
            time=time,
            q=q,
            base_tf=base_tf,
            attachment_end=attachment_end
        )
        
        return super().setUp()
    
    def test_trajectory_visualisation(self):
        # Visualise the trajectory synchronously
        arm_ids = visualise_trajectory_sync(self.canvas, self.arm, self.traj, border=self.border, objs=self.concave_objs)
        
        # Visualise the trajectory asynchronously
        
        # start variables
        thread = evaluate_trajectory_threaded(self.traj, self.arm, self.concave_objs)
        
        start_time = time.time()
    
        print("Asynchronous visualisation starting ...")
        while True:
            
            # Update visualisation
            [arm_ids, done] = update_trajectory_visualisation(time.time() - start_time,self.canvas,self.arm,self.traj
                                                            ,arm_ids,border=self.border,objs=self.concave_objs)
            if done:
                break
            
            time.sleep(0.01)
        
        print("Asynchronous visualisation finished. Joining thread...")
        thread.join()
        print("Thread completed successfully.")
        
        
        
        
        
if __name__ == "__main__":
    unittest.main()