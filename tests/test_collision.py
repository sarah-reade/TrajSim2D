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
from trajsim2d_core.collision import detect_any_collisions_AABB, get_AABB, crop_shape_to_AABB, convert_shape_to_numpy, suthHodgClip, detect_collision_DISTANCE


class TestCollisionDetection(unittest.TestCase):
    """
    @class TestCollisionDetection
    @brief Test suite for verifying AABB-related visualisation and collision utilities.
    """

    # -------- CLASS-LEVEL SHARED VARIABLES --------
    border = None
    objs = None
    arm = None
    canvas = None
    base_tf = None
    config_1 = None
    border_id = None
    object_ids = None
    arm_ids = None

    # ----------------------------------------------

    @classmethod
    def setUpClass(cls):
        """
        Runs ONCE before any tests.
        Creates the shared environment: border, objects, arm, canvas.
        """
        print("\n[SETUP] Initialising shared test environment...")

        # Generate a random bumpy border
        cls.border = generate_random_border(border_size=10, smoothness=0.1)

        # Generate random objects
        cls.objs = generate_random_objects(object_size=0.5, num_objs=20, smoothness=0.1)

        # Generate manipulator + random config
        cls.arm = PlanarManipulator()
        cls.config_1 = cls.arm.generate_random_config(cls.border, cls.objs, 1)

        # Initialise shared canvas & drawing elements
        (cls.canvas,
         cls.base_tf,
         cls.border_id,
         cls.object_ids,
         cls.arm_ids) = initialise_visualisation(
            border=cls.border,
            objs=cls.objs,
            arm=cls.arm,
            joint_config_1=cls.config_1
        )

        cls.arm_geometry = cls.arm.make_arm_geometry(cls.config_1, cls.base_tf)

        cls.temp_shapes = []  # to track temporary shapes added during tests

        print("[SETUP COMPLETE] Shared vars initialised.\n")

   

    # =====================================================
    # =============== SHAPE CROPPING TEST =================
    # =====================================================

    def test_shape_cropping(self):
        """
        @brief Test shape cropping visualisation.
               This test should ONLY display a cropped shape.
        """

        for shape in self.temp_shapes:
            self.canvas.remove_shape(shape)
        self.temp_shapes = []

        inflation = 0.1
        canvas = self.canvas 

        # get overlapping bounds
        collision_flag, collision_list = detect_any_collisions_AABB(
            self.arm_geometry, self.objs, inflation
        )

        if collision_flag:
            for (shape_1, shape_2, collision_AABB) in collision_list:
                area = collision_AABB

                # convert all shapes into np.ndarray of points
                shape_1 = convert_shape_to_numpy(shape_1)
                shape_2 = convert_shape_to_numpy(shape_2)

                shape_1 = crop_shape_to_AABB(shape_1, area)
                shape_2 = crop_shape_to_AABB(shape_2, area)
                try:
                    self.temp_shapes.append(self.canvas.add_shape(shape_1, color='blue'))
                    self.temp_shapes.append(self.canvas.add_shape(shape_2, color='green'))
                except Exception as e:
                    print(f"Error adding cropped shapes to canvas: {e}")
        else:
            print("No collisions detected for cropping test.")


        
        print("Paused at end of Shape cropping Visualisation. Press Enter to continue...")
        input()  # Waits until the user presses Enter

        print("Continuing execution...")


        self.assertTrue(True, "Shape cropping visualisation closed successfully.")

    def test_sutherland_hodgman_cropping(self):
        # Defining polygon vertices in clockwise order
        poly_size = 3
        poly_points = np.array([[0.1, 0.15], [0.2, 0.25], [0.3, 0.2]])

        # Defining clipper polygon vertices in clockwise order
        # 1st Example with square clipper
        clipper_size = 4
        clipper_points = np.array([[0.15, 0.15], [0.15, 0.2], [0.2, 0.2], [0.2, 0.15]])

        # 2nd Example with triangle clipper
        # clipper_size = 3
        # clipper_points = np.array([[100,300], [300,300], [200,100]])

        # Calling the clipping function
        cropped_poly = suthHodgClip(poly_points, poly_size, clipper_points, clipper_size)

        self.temp_shapes.append(self.canvas.add_shape(poly_points, color='gray'))          # Original polygon
        self.temp_shapes.append(self.canvas.add_shape(clipper_points, color='orange'))     # Clipper polygon
        self.temp_shapes.append(self.canvas.add_shape(cropped_poly, color='purple'))       # Cropped polygon
        

        plt.show(block=False)

        print("Paused at end of Sutherland_Hodgman cropping Visualisation. Press Enter to continue...")
        input()  # Waits until the user presses Enter

        print("Continuing execution...")


        answer = [[0.2, 0.174], [0.15, 0.162], [0.15, 0.2], [0.15, 0.2], [0.2, 0.2]]
        self.assertTrue(np.allclose(cropped_poly, answer, atol=0.001))

    # =====================================================
    # =============== AABB VISUALISATION TEST =============
    # =====================================================

    def test_AABB_visualisation(self):
        """
        @brief Draw AABBs for all objects and arm links,
               plus collision AABBs if any.
        """

        for shape in self.temp_shapes:
            self.canvas.remove_shape(shape)
        self.temp_shapes = []

        inflation = 0.1

        # ------------------ AABB for OBJECTS ------------------

        for obj in self.objs:
            minx, miny, maxx, maxy = get_AABB(obj, inflation)

            area = np.array([
                [minx, miny],
                [minx, maxy],
                [maxx, maxy],
                [maxx, miny],
            ])
            self.temp_shapes.append(self.canvas.add_shape(area, color='black'))

        # ------------------ AABB for ARM PARTS ------------------

        

        for part in self.arm_geometry:
            minx, miny, maxx, maxy = get_AABB(part, inflation)

            area = np.array([
                [minx, miny],
                [minx, maxy],
                [maxx, maxy],
                [maxx, miny],
            ])
            self.temp_shapes.append(self.canvas.add_shape(area, color='gray'))

        # ------------------ COLLISIONS ------------------

        collision_flag, collision_list = detect_any_collisions_AABB(
            self.arm_geometry, self.objs, inflation
        )

        if collision_flag:
            for (_, _, collision_AABB) in collision_list:
                minx, miny, maxx, maxy = collision_AABB
                area = np.array([
                    [minx, miny],
                    [minx, maxy],
                    [maxx, maxy],
                    [maxx, miny],
                ])
                self.temp_shapes.append(self.canvas.add_shape(area, color='red'))
        else:
            print("No collisions detected.")

        plt.show(block=False)

        print("Paused at end of AABB Visualisation. Press Enter to continue...")
        input()  # Waits until the user presses Enter

        print("Continuing execution...")

        self.assertTrue(True, "AABB visualisation completed with no errors.")


    # =====================================================
    # ================ GJK VISUALISATION TEST =============
    # =====================================================

    def test_GJK_distance_visualisation(self):
        """
        @brief Visualise GJK distance between two shapes.
        """
        for shape in self.temp_shapes:
            print(f"Removing temporary shape from canvas: {shape}")
            self.canvas.remove_shape(shape)
        self.temp_shapes = []

        inflation = 0.2
        canvas = self.canvas  # shorthand

        collision_flag, collision_list = detect_any_collisions_AABB(
            self.arm_geometry, self.objs, inflation
        )

        if not collision_flag:
            self.skipTest("No collisions detected; skipping GJK distance visualisation.")


        for (shape_1, shape_2, intersection) in collision_list:
            collision, distance = detect_collision_DISTANCE(shape_1, shape_2,intersection,'GJK')
            if collision:
                print("Shapes are colliding; GJK distance is 0.")
                self.temp_shapes.append(self.canvas.add_shape(shape_1, color='red'))
                self.temp_shapes.append(self.canvas.add_shape(shape_2, color='red'))
                continue
            print(f"GJK Distance: {distance}")
            self.temp_shapes.append(self.canvas.add_shape(shape_1, color='orange'))
            self.temp_shapes.append(self.canvas.add_shape(shape_2, color='purple'))


        plt.show(block=False)
        print("Paused at end of GJK Visualisation. Press Enter to continue...")
        input()  # Waits until the user presses Enter

        print("Continuing execution...")


        self.assertTrue(True, "GJK completed with no errors.")

if __name__ == "__main__":
    unittest.main()
