###############################################################################
## @file test_utils.py
## @brief Test Utility functions for TrajSim2D.
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
import numpy as np
from trajsim2d_core.utils import calc_array_diff

class TestCalcArrayDiff(unittest.TestCase):

    def test_list_input(self):
        """Test with a simple Python list."""
        result = calc_array_diff([1.0, 2.5, 4.0, 7.3])
        expected = [1.5, 1.5, 3.3]
        np.testing.assert_allclose(result, expected, rtol=1e-9)

    def test_numpy_array_input(self):
        """Test with a NumPy array input."""
        arr = np.array([1.0, 2.5, 4.0, 7.3])
        result = calc_array_diff(arr)
        expected = np.array([1.5, 1.5, 3.3])
        np.testing.assert_allclose(result, expected, rtol=1e-9)
        self.assertIsInstance(result, np.ndarray)

    def test_single_element(self):
        """Test behavior with a single-element list."""
        result = calc_array_diff([5.0])
        expected = []
        self.assertEqual(result, expected)

    def test_empty_input(self):
        """Test behavior with an empty input."""
        result = calc_array_diff([])
        expected = []
        self.assertEqual(result, expected)

    def test_negative_values(self):
        """Test with descending values."""
        result = calc_array_diff([5.0, 3.0, 1.0])
        expected = [-2.0, -2.0]
        np.testing.assert_allclose(result, expected, rtol=1e-9)


if __name__ == "__main__":
    unittest.main()