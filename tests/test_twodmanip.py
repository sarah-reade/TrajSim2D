###############################################################################
## @file test_visualisation.py
## @brief Test Visualisation functions for TrajSim2D.
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
## >>> pytest tests/test_twodmanip.py
###############################################################################

import unittest
import numpy as np
from trajsim2d_core.twodmanip import PlanarManipulator

class TestPlanarManipulatorKinematics(unittest.TestCase):
    """
    @brief Unit tests for PlanarManipulator forward kinematics methods.
    @details
    This test suite verifies the correctness of the forward kinematics calculations
    for a simple 2D planar manipulator. It ensures the generated transformations
    are consistent with geometric expectations when given specific joint configurations.
    """

    def setUp(self):
        """
        @brief Initialize a simple planar manipulator for testing.
        @details
        Creates a 2-link planar manipulator with fixed dimensions and no base offset.
        The manipulator links are aligned along the Y-axis at zero configuration.
        """
        self.link_lengths = np.array([1.0, 1.0])
        self.link_width = np.array([0.1, 0.1])
        self.joint_radius = 0.05
        self.base_offset = 0.0
        self.base_tf = np.eye(3)

        self.manipulator = PlanarManipulator(
            base_tf=self.base_tf,
            base_offset=self.base_offset,
            link_width=self.link_width,
            link_lengths=self.link_lengths,
            joint_radius=self.joint_radius
        )

    def test_forward_kinematics_zero_angles(self):
        """
        @test
        @brief Test forward kinematics with all joint angles set to zero.
        @details
        For zero joint angles, the manipulator should align along the +Y axis.
        The end effector position should be at Y = sum(link_lengths).
        """
        config = np.array([0.0, 0.0])
        tfs = self.manipulator.forward_kinematics(self.base_tf, config)

        # End effector transform
        end_effector_tf = tfs[-1]
        expected_y = np.sum(self.link_lengths)

        self.assertAlmostEqual(end_effector_tf[1, 2], expected_y, places=6)
        self.assertAlmostEqual(end_effector_tf[0, 2], 0.0, places=6)

    def test_forward_kinematics_right_angle(self):
        """
        @test
        @brief Test forward kinematics with a 90-degree rotation at the first joint.
        @details
        The first link rotates into the +X direction; the second link also extends along +X
        since its joint angle is 0 relative to the first link’s frame.
        The end effector should therefore be at (2.0, 0.0).
        """
        config = np.array([np.pi / 2, 0.0])
        tfs = self.manipulator.forward_kinematics(self.base_tf, config)

        end_effector_tf = tfs[-1]
        expected_x = np.sum(self.link_lengths)
        expected_y = 0.0

        self.assertAlmostEqual(end_effector_tf[0, 2], expected_x, places=6)
        self.assertAlmostEqual(end_effector_tf[1, 2], expected_y, places=6)

    def test_link_forward_kinematics(self):
        """
        @test
        @brief Test computation of link center transforms.
        @details
        Ensures the link transforms are halfway along each link’s length.
        """
        config = np.array([0.0, 0.0])
        tfs = self.manipulator.forward_kinematics(self.base_tf, config)
        link_tfs = self.manipulator.link_forward_kinematics(tfs)

        # Base link center should be below base offset
        base_link_tf = link_tfs[0]
        self.assertLess(base_link_tf[1, 2], 0.0)

        # First link center should be halfway along Y
        first_link_tf = link_tfs[1]
        expected_y = self.link_lengths[0] / 2
        self.assertAlmostEqual(first_link_tf[1, 2], expected_y, places=6)


if __name__ == '__main__':
    unittest.main()
