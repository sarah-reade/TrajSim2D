import unittest
import numpy as np
from trajsim2d_core.environment import generate_random_objects, generate_random_edge_point, generate_random_convex_objects

class TestRandomObjects(unittest.TestCase):
    def test_generate_random_objects_basic(self):
        num_points = 50
        num_objs = 5
        object_size = 1.0
        smoothness = 0.8

        objs = generate_random_objects(
            num_points=num_points,
            object_size=object_size,
            num_objs=num_objs,
            smoothness=smoothness
        )

        # Check return type
        self.assertIsInstance(objs, list)
        self.assertEqual(len(objs), num_objs)

        # Check each object
        for obj in objs:
            self.assertIsInstance(obj, np.ndarray)
            self.assertEqual(obj.shape, (num_points, 2))
            self.assertTrue(np.all(np.isfinite(obj)))  # no NaNs or infinities

    def test_generate_random_convex_objects_basic(self):
        num_points = 50
        num_objs = 5
        object_size = 1.0
        smoothness = 0.8

        convex_objs = generate_random_convex_objects(
            num_points=num_points,
            object_size=object_size,
            num_objs=num_objs,
            smoothness=smoothness
        )

        # Check return type
        self.assertIsInstance(convex_objs, list)
        self.assertGreater(len(convex_objs), 0)   # may be > num_objs

        # Check each convex part
        for part in convex_objs:
            self.assertIsInstance(part, np.ndarray)
            self.assertEqual(part.shape[1], 2)        # Nx2, N may vary
            self.assertGreaterEqual(part.shape[0], 3) # convex polygon → minimum 3 points
            self.assertTrue(np.all(np.isfinite(part)))

    def test_generate_random_objects_defaults(self):
        # Call with defaults (randomized)
        objs = generate_random_objects()
        self.assertIsInstance(objs, list)
        self.assertTrue(len(objs) > 0)
        for obj in objs:
            self.assertIsInstance(obj, np.ndarray)
            self.assertEqual(obj.shape[1], 2)  # at least two columns

    def test_generate_random_edge_point_none(self):
        # Both border and objs are None
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
        # Only objs provided
        obj1 = np.array([[0,0],[1,0],[1,1],[0,1]])
        obj2 = np.array([[2,2],[3,2],[3,3],[2,3]])
        objs = [obj1, obj2]
        
        point, angle = generate_random_edge_point(objs=objs)
        
        self.assertIsInstance(point, np.ndarray)
        self.assertEqual(point.shape, (2,))
        self.assertIsInstance(angle, float)
        self.assertTrue(np.isfinite(angle))
        # Point should be in one of the objs
        self.assertTrue(any(any(np.all(point == p) for p in obj) for obj in objs))

    def test_generate_random_edge_point_border_objects(self):
        # Both border and objs provided
        border = np.array([[0,0],[1,0],[1,1],[0,1]])
        obj1 = np.array([[2,2],[3,2],[3,3],[2,3]])
        objs = [obj1]
        
        # Run multiple times to exercise random choice
        for _ in range(10):
            result = generate_random_edge_point(border=border, objs=objs)
            self.assertIsInstance(result, tuple)
            point, angle = result
            self.assertIsInstance(point, np.ndarray)
            self.assertEqual(point.shape, (2,))
            self.assertIsInstance(angle, float)
            self.assertTrue(np.isfinite(angle))
            # Point should belong to either border or one of the objs
            in_border = any(np.all(point == p) for p in border)
            in_objects = any(any(np.all(point == p) for p in obj) for obj in objs)
            self.assertTrue(in_border or in_objects)



if __name__ == "__main__":
    unittest.main()