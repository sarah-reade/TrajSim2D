import unittest
import numpy as np
from trajsim2d_core.environment import generate_random_objects, generate_random_edge_point

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

    def test_generate_random_edge_point_none(self):
        # Both border and objects are None
        result = generate_random_edge_point()
        self.assertIsNone(result)

    def test_generate_random_edge_point_border(self):
        # Only border provided
        border = np.array([[0,0],[1,0],[1,1],[0,1]])
        point, angle = generate_random_edge_point(border=border)
        
        self.assertIsInstance(point, np.ndarray)
        self.assertEqual(point.shape, (2,))
        self.assertIsInstance(angle, float)
        self.assertTrue(np.isfinite(angle))
        # Point should be one of the border points
        self.assertTrue(any(np.all(point == p) for p in border))

    def test_generate_random_edge_point_objects(self):
        # Only objects provided
        obj1 = np.array([[0,0],[1,0],[1,1],[0,1]])
        obj2 = np.array([[2,2],[3,2],[3,3],[2,3]])
        objects = [obj1, obj2]
        
        point, angle = generate_random_edge_point(objects=objects)
        
        self.assertIsInstance(point, np.ndarray)
        self.assertEqual(point.shape, (2,))
        self.assertIsInstance(angle, float)
        self.assertTrue(np.isfinite(angle))
        # Point should be in one of the objects
        self.assertTrue(any(any(np.all(point == p) for p in obj) for obj in objects))

    def test_generate_random_edge_point_border_objects(self):
        # Both border and objects provided
        border = np.array([[0,0],[1,0],[1,1],[0,1]])
        obj1 = np.array([[2,2],[3,2],[3,3],[2,3]])
        objects = [obj1]
        
        # Run multiple times to exercise random choice
        for _ in range(10):
            result = generate_random_edge_point(border=border, objects=objects)
            self.assertIsInstance(result, tuple)
            point, angle = result
            self.assertIsInstance(point, np.ndarray)
            self.assertEqual(point.shape, (2,))
            self.assertIsInstance(angle, float)
            self.assertTrue(np.isfinite(angle))
            # Point should belong to either border or one of the objects
            in_border = any(np.all(point == p) for p in border)
            in_objects = any(any(np.all(point == p) for p in obj) for obj in objects)
            self.assertTrue(in_border or in_objects)



if __name__ == "__main__":
    unittest.main()