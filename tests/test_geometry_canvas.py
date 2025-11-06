##
# @file test_geometry_canvas.py
# @brief Unit tests for the GeometryCanvas library.
# @details
# This file contains automated tests for the GeometryCanvas class, verifying
# shape creation, manipulation, and rendering logic using Matplotlib.
#
# Run with:
# @code
# python -m unittest test_geometry_canvas.py
# @endcode
#
# @authors
# Your Name (you@example.com)
# ChatGPT (OpenAI Assistant)
#
# @date
# 2025-10-30
#
# @license
# MIT License
##

import unittest
import numpy as np
import matplotlib.pyplot as plt
from trajsim2d_core.geometry_canvas import GeometryCanvas
from matplotlib.patches import Rectangle, Circle

class TestGeometryCanvasVisual(unittest.TestCase):
    """ @class TestGeometryCanvasVisual
        @brief Visual and functional tests for GeometryCanvas using unittest.
    """

    def setUp(self):
        """ @brief Create a new GeometryCanvas for each test. """
        self.canvas = GeometryCanvas()

    def pause(self, seconds=1):
        """ @brief Pause to visually inspect the canvas. """
        plt.pause(seconds)

    def test_add_shapes_and_visualize(self):
        """ @brief Add rectangle, circle, polygon, and display visually. """
        self.pause(1)
        # Add rectangle
        rect_id = self.canvas.add_rectangle((1, 1), 3, 2, color='red', alpha=0.5)
        self.assertIn(rect_id, self.canvas.shapes)
        self.pause(1)

        # Add circle
        circ_id = self.canvas.add_circle((6, 4), 1.5, color='green', alpha=0.5)
        self.assertIn(circ_id, self.canvas.shapes)
        self.pause(1)

        # Add polygon
        points = np.array([[2, 6], [4, 6], [3, 8]])
        poly_id = self.canvas.add_polygon(points, color='blue', alpha=0.6)
        self.assertIn(poly_id, self.canvas.shapes)
        self.pause(1)

        # Move shapes
        self.canvas.move_shape(rect_id, dx=2, dy=1)
        self.canvas.move_shape(circ_id, dx=-1, dy=-0.5)
        self.pause(1)

        # Update color
        self.canvas.update_color(poly_id, 'orange')
        self.pause(1)

        # Add array as a polygon shape
        arr_shape = np.array([
            [5.0, 5.0],
            [7.0, 5.0],
            [8.0, 7.0],
            [6.0, 7.0]
        ], dtype=float)
        arr_id = self.canvas.add_array(arr_shape, color='magenta', alpha=0.7)
        self.assertIn(arr_id, self.canvas.shapes)
        self.pause(1)
        
        # Remove a shape
        self.canvas.remove_shape(rect_id)
        self.assertNotIn(rect_id, self.canvas.shapes)
        self.pause(1)

        print("Visual unittest complete. Close the window to exit.")
        plt.show(block=True)

    def test_add_rectangle_with_transform(self):
        """Test adding a rectangle using a 3×3 homogeneous transform."""
        angle = np.pi / 4
        cos_a, sin_a = np.cos(angle), np.sin(angle)
        tf = np.array([
            [cos_a, -sin_a, 2.0],
            [sin_a,  cos_a, 3.0],
            [0, 0, 1]
        ])

        shape_id = self.canvas.add_rectangle(tf, 2, 1, color='blue', alpha=0.7)
        self.assertIsInstance(shape_id, str)
        self.assertIn(shape_id, self.canvas.shapes)

        self.pause()

    def test_add_circle_with_transform(self):
        """Test adding a circle using a 3×3 transform matrix (translation only)."""
        tf = np.array([
            [1, 0, 5.0],
            [0, 1, 6.0],
            [0, 0, 1]
        ])

        shape_id = self.canvas.add_circle(tf, 2.0, color='purple', alpha=0.8)
        self.assertIn(shape_id, self.canvas.shapes)

        circ = self.canvas.shapes[shape_id]
        self.assertIsInstance(circ, Circle)
        self.assertAlmostEqual(circ.center[0], 5.0)
        self.assertAlmostEqual(circ.center[1], 6.0)
        self.assertAlmostEqual(circ.get_radius(), 2.0)

        self.pause()
if __name__ == "__main__":
    unittest.main()