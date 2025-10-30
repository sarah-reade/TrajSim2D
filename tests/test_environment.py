import unittest
import numpy as np
from trajsim2d_core.environment import generate_random_objects

class TestRandomObjects(unittest.TestCase):
    def test_generate_random_objects_basic(self):
        num_points = 50
        num_objects = 5
        object_size = 1.0
        smoothness = 0.8

        objects = generate_random_objects(
            num_points=num_points,
            object_size=object_size,
            num_objects=num_objects,
            smoothness=smoothness
        )

        # Check return type
        self.assertIsInstance(objects, list)
        self.assertEqual(len(objects), num_objects)

        # Check each object
        for obj in objects:
            self.assertIsInstance(obj, np.ndarray)
            self.assertEqual(obj.shape, (num_points, 2))
            self.assertTrue(np.all(np.isfinite(obj)))  # no NaNs or infinities

    def test_generate_random_objects_defaults(self):
        # Call with defaults (randomized)
        objects = generate_random_objects()
        self.assertIsInstance(objects, list)
        self.assertTrue(len(objects) > 0)
        for obj in objects:
            self.assertIsInstance(obj, np.ndarray)
            self.assertEqual(obj.shape[1], 2)  # at least two columns

if __name__ == "__main__":
    unittest.main()