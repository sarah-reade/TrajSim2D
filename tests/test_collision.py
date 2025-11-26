###############################################################################
## @file test_collision.py
## @brief Unit tests for collision detection and AABB visualisation utilities.
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
import numpy as np


from trajsim2d_core.environment import generate_random_border, generate_random_objects
from trajsim2d_core.visualisation import initialise_visualisation
from trajsim2d_core.twodmanip import PlanarManipulator
from trajsim2d_core.collision import detect_any_collisions_AABB, get_AABB


class TestCollisionDetection(unittest.TestCase):
    """
    @class TestCollisionDetection
    @brief Test suite for verifying AABB-related visualisation and collision utilities.
    """

    def test_AABB_visualisation(self):
        """
        @brief Test initialisation and user-confirmation of the AABB visualisation.

        This test sets up a visualisation containing:
        - a random border,
        - random objects,
        - a planar manipulator arm.

        The figure is displayed in blocking mode and the test passes once the
        user closes the window.
        """

        # --- Setup identical to the original test ---

        # Generate a random bumpy border
        border = generate_random_border(border_size=10, smoothness=0.1)

        # Generate a set of random bumpy objects
        objs = generate_random_objects(object_size=0.5, num_objs=20, smoothness=0.1)

        # Generate a random manipulator arm
        arm = PlanarManipulator()

        config_1 = arm.generate_random_config(border,objs,1)

        # Initialise the visualisation canvas and drawable elements
        canvas, base_tf, border_id, object_ids, arm_ids = initialise_visualisation(
            border=border, objs=objs, arm=arm, joint_config_1=config_1
        )

        inflation = 0.1
        # Display AABB
        for obj in objs:
            obj_AABB = get_AABB(obj,inflation)
            minx, miny, maxx, maxy = obj_AABB

            area = np.array([
                [minx, miny],
                [minx, maxy],
                [maxx, maxy],
                [maxx, miny]
            ], dtype=float)

            canvas.add_shape(area,color='black')


        arm_geometry = arm.make_arm_geometry(config_1,base_tf)
        for part in arm_geometry:
            part_AABB = get_AABB(part,inflation)
            minx, miny, maxx, maxy = part_AABB

            area = np.array([
                [minx, miny],
                [minx, maxy],
                [maxx, maxy],
                [maxx, miny]
            ], dtype=float)

            canvas.add_shape(area,color='gray')



        # Get collisions
        collision_flag, collision_list = detect_any_collisions_AABB(arm_geometry,objs,inflation)

        # Display collisions
        if collision_flag:
            for collision in collision_list:
                shape_1, shape_2, collision_AABB = collision
                minx, miny, maxx, maxy = collision_AABB

                area = np.array([
                    [minx, miny],
                    [minx, maxy],
                    [maxx, maxy],
                    [maxx, miny]
                ], dtype=float)

                canvas.add_shape(area,color='red')

        else:
            print("No Collisions Detected")

        # Display figure in blocking mode so user must close the window
        plt.show(block=True)

        # If execution reaches here, the window was closed successfully
        self.assertTrue(True, msg="AABB visualisation closed successfully.")


if __name__ == "__main__":
    unittest.main()
