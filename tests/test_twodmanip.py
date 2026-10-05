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
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from trajsim2d_core.twodmanip import PlanarManipulator

TEST_OUTPUT_DIR = Path(__file__).parent / "test_outputs"


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


class TestPlanarManipulatorUrdfPersistence(unittest.TestCase):
    """
    @brief Unit tests for saving and loading PlanarManipulator URDF files.
    """

    @classmethod
    def setUpClass(cls):
        """Create the persistent test output directory once per test class."""
        TEST_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    def setUp(self):
        """Create a manipulator with explicit values for round-trip testing."""
        self.base_tf = np.array([
            [0.0, -1.0, 1.5],
            [1.0, 0.0, -0.25],
            [0.0, 0.0, 1.0],
        ])
        self.manipulator = PlanarManipulator(
            base_tf=self.base_tf,
            base_offset=0.2,
            link_width=0.15,
            link_lengths=np.array([1.0, 0.75]),
            joint_radius=0.05,
            link_masses=np.array([0.4, 1.2, 0.8]),
            n=2,
            max_velocity=0.35,
            adhesion=2.1,
            ee_width=0.4,
            friction=0.7,
        )

    def test_save_and_load_preserves_manipulator_attributes(self):
        """
        @test
        @brief Test that saving and loading preserves all saved arm attributes.
        """
        filename = TEST_OUTPUT_DIR / "manipulator_roundtrip.urdf"
        self.manipulator.save_to_urdf(filename)
        loaded = PlanarManipulator.load_from_urdf(filename)

        self.assertEqual(loaded.n, self.manipulator.n)
        self.assertEqual(loaded.base_offset, self.manipulator.base_offset)
        self.assertEqual(loaded.link_width, self.manipulator.link_width)
        self.assertEqual(loaded.joint_radius, self.manipulator.joint_radius)
        np.testing.assert_allclose(
            loaded.link_lengths, self.manipulator.link_lengths
        )
        np.testing.assert_allclose(
            loaded.link_masses, self.manipulator.link_masses
        )
        np.testing.assert_allclose(loaded.base_tf, self.manipulator.base_tf)
        self.assertEqual(
            loaded.joint_limits.position,
            self.manipulator.joint_limits.position,
        )
        self.assertEqual(
            loaded.joint_limits.velocity,
            self.manipulator.joint_limits.velocity,
        )
        self.assertEqual(
            loaded.joint_limits.torque,
            self.manipulator.joint_limits.torque,
        )
        self.assertEqual(
            loaded.end_effector.adhesion,
            self.manipulator.end_effector.adhesion,
        )
        self.assertEqual(
            loaded.end_effector.width,
            self.manipulator.end_effector.width,
        )
        self.assertEqual(
            loaded.end_effector.friction,
            self.manipulator.end_effector.friction,
        )

    def test_constructor_loads_from_urdf_filename(self):
        """
        @test
        @brief Test constructor-based loading from a URDF filename.
        """
        filename = TEST_OUTPUT_DIR / "manipulator_constructor.urdf"
        self.manipulator.save_to_urdf(filename)
        loaded = PlanarManipulator(filename=filename)

        self.assertEqual(loaded.n, self.manipulator.n)
        np.testing.assert_allclose(
            loaded.link_lengths, self.manipulator.link_lengths
        )
        self.assertEqual(
            loaded.end_effector.friction,
            self.manipulator.end_effector.friction,
        )

    def test_saved_urdf_contains_thin_clipped_geometry(self):
        filename = TEST_OUTPUT_DIR / "manipulator_geometry.urdf"
        self.manipulator.save_to_urdf(filename)
        root = ET.parse(filename).getroot()

        base_box = root.find("./link[@name='base_link']/visual/geometry/box")
        self.assertIsNotNone(base_box)
        self.assertEqual(base_box.attrib["size"], "0.15 0.15000000000000002 0.01")

        first_link = root.find("./link[@name='link_1']")
        self.assertIsNotNone(first_link)
        boxes = first_link.findall("./visual/geometry/box")
        cylinders = first_link.findall("./visual/geometry/cylinder")
        self.assertEqual(len(boxes), 1)
        self.assertEqual(len(cylinders), 1)
        self.assertEqual(boxes[0].attrib["size"], "0.15 0.9 0.01")
        self.assertEqual(cylinders[0].attrib["radius"], "0.05")
        self.assertEqual(cylinders[0].attrib["length"], "0.01")

        last_box = root.find("./link[@name='link_2']/visual/geometry/box")
        self.assertIsNotNone(last_box)
        self.assertEqual(last_box.attrib["size"], "0.15 0.7 0.01")

        joint_origin = root.find("./joint[@name='joint_2']/origin")
        self.assertIsNotNone(joint_origin)
        self.assertEqual(joint_origin.attrib["xyz"], "0 1.0 0")


if __name__ == '__main__':
    unittest.main()
