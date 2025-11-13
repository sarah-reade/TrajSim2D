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
from trajsim2d_core.utils import calc_array_diff, generate_random_number,generate_random_int,generate_random_int_array, tangent_angle_at_point, normal_angle_at_point

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

class TestGenerateRandomNumber(unittest.TestCase):
    """Tests for the generate_random_number() function with printouts."""

    def test_output_type_and_bounds(self):
        min_val = 1.0
        max_val = 5.0
        val = generate_random_number(min_val, max_val)
        print(f"Generated random number: {val} (expected between {min_val} and {max_val})")
        self.assertIsInstance(val, float)
        self.assertGreaterEqual(val, min_val)
        self.assertLess(val, max_val)

    def test_multiple_calls(self):
        min_val = 0.0
        max_val = 20.0
        val1 = generate_random_number(min_val, max_val)
        val2 = generate_random_number(min_val, max_val)
        print(f"First call: {val1}, Second call: {val2}")
        # It's extremely unlikely two consecutive random floats are exactly equal
        self.assertNotEqual(val1, val2)

    def test_generate_random_int(self):
        min_val = 1
        max_val = 10
        val = generate_random_int(min_val, max_val)
        print(f"Generated random int: {val}")
        self.assertIsInstance(val, int)
        self.assertGreaterEqual(val, min_val)
        self.assertLess(val, max_val)

    def test_generate_random_int_array(self):
        low = 5
        high = 15
        size = 10
        arr = generate_random_int_array(low, high, size)
        print(f"Generated random int array: {arr}")
        self.assertIsInstance(arr, np.ndarray)
        self.assertEqual(arr.shape[0], size)
        self.assertTrue(np.all(arr >= low))
        self.assertTrue(np.all(arr < high))

        
if __name__ == "__main__":
    unittest.main()