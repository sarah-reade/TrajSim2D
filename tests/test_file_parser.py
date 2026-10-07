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
import csv
import json
from pathlib import Path
from trajsim2d_core.file_parser import (
    load_canvas_from_file,
    save_canvas_to_file,
    save_trajectory_to_file,
)
from trajsim2d_core.calculations import Trajectory, evaluate_trajectory
from trajsim2d_core.twodmanip import PlanarManipulator
import numpy as np

TEST_OUTPUT_DIR = Path(__file__).parent / "test_outputs"


class TestSaveTrajectory(unittest.TestCase):
    def setUp(self):
        TEST_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

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
        filename = TEST_OUTPUT_DIR / "trajectory_output"
        save_trajectory_to_file(str(filename), self.traj, self.manip)

        with open(Path(filename) / "trajectory.csv", newline="") as file:
            rows = list(csv.reader(file))

        self.assertEqual(
            rows[0][-3:],
            [
                "qdotdot_exceeded",
                "tau_exceeded",
                "adhesion_exceeded",
            ],
        )
        
        self.assertEqual(len(rows), len(self.time) + 1)
        for row, in_collision, qdotdot_exceeded, tau_exceeded, adhesion_exceeded in zip(
            rows[1:],
            self.traj.in_collision,
            self.traj.qdotdot_exceeded,
            self.traj.tau_exceeded,
            self.traj.adhesion_exceeded,
        ):
            self.assertEqual(row[-4], str(in_collision))
            self.assertEqual(row[-3], str(qdotdot_exceeded))
            self.assertEqual(row[-2], str(tau_exceeded))
            self.assertEqual(row[-1], str(adhesion_exceeded))

        with open(Path(filename) / "metadata.json", encoding="utf-8") as file:
            metadata = json.load(file)
        self.assertEqual(
            metadata["manipulator_parameters"]["link_lengths"],
            self.manip.link_lengths,
        )
            

    def test_save_and_load_canvas_with_arm(self):
        border = np.array([[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]])
        obstacles = [
            np.array([[1.0, 1.0], [1.5, 1.0], [1.5, 1.5], [1.0, 1.5]]),
            np.array([[2.0, 2.0], [2.5, 2.0], [2.25, 2.5]]),
        ]
        base_transform = np.array([
            [0.0, -1.0, 2.0],
            [1.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
        ])
        start_config = np.array([0.1])
        end_config = np.array([-0.2])

        canvas_path = TEST_OUTPUT_DIR / "example_canvas.canvas"
        save_canvas_to_file(
            canvas_path,
            self.manip,
            border=border,
            obstacles=obstacles,
            base_transform=base_transform,
            start_config=start_config,
            end_config=end_config,
        )

        loaded = load_canvas_from_file(canvas_path)

        self.assertTrue(canvas_path.with_suffix(".urdf").exists())
        np.testing.assert_allclose(loaded.border, border)
        self.assertEqual(len(loaded.obstacles), len(obstacles))
        for actual, expected in zip(loaded.obstacles, obstacles):
            np.testing.assert_allclose(actual, expected)
        np.testing.assert_allclose(loaded.base_transform, base_transform)
        np.testing.assert_allclose(loaded.arm.base_tf, base_transform)
        np.testing.assert_allclose(loaded.start_config, start_config)
        np.testing.assert_allclose(loaded.end_config, end_config)
        np.testing.assert_allclose(
            loaded.arm.link_lengths, self.manip.link_lengths
        )
        
        