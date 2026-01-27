###############################################################################
## @file test_file_parser.py
## @brief Test File Parser functions for TrajSim2D.
##
## This file is part of the TrajSim2D project, a 2D planar manipulator simulator
## for trajectory planning, collision testing, and environment visualization.
## 
## Author: Sarah Reade
## Email: 28378329@students.lincoln.ac.uk
## Co-Author: ChatGPT (GPT-5 by OpenAI)
## Date: 2025-10-23
## Version: 0.0.1
##
## License: MIT
##
## Usage:
## >>> pytest tests/test_file_parser.py
###############################################################################

import unittest
from trajsim2d_core.file_parser import save_trajectory_to_file
from trajsim2d_core.calculations import Trajectory, evaluate_trajectory
from trajsim2d_core.twodmanip import PlanarManipulator
import numpy as np

class TestSaveTrajectory(unittest.TestCase):
    def setUp(self):
        # Simple 1-link manipulator
        self.link_length = [2.0]
        self.link_mass = [0.4, 1.5]  # last is EE
        self.base_offset = 0.5
        self.g = -9.81

        # Create a simple PlanarManipulator
        self.manip = PlanarManipulator(
            n=1,
            base_offset=self.base_offset,
            link_lengths=self.link_length,
            link_masses=self.link_mass
        )

        # Simple trajectory: 3 time points
        self.time = np.array([0.0, 0.1, 0.2])
        self.q = np.zeros((3, 1))  # joint at 0 radians
        self.base_tf = np.eye(3)
        self.attachment_end = None

        # Trajectory object
        self.traj = Trajectory(
            time=self.time,
            q=self.q,
            base_tf=self.base_tf,
            attachment_end=self.attachment_end
        )

    def test_save_trajectory(self):
        # Evaluate trajectory
        evaluate_trajectory(self.traj, self.manip)

        # Save to file
        filename = "/tmp/test_trajectory"
        save_trajectory_to_file(filename, self.traj, self.manip)
        
        