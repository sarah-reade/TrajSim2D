###############################################################################
## @file collision.py
## @brief Collision checking functions for TrajSim2D.
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
## >>> import trajsim2d_core.collision as collision
###############################################################################

# Imports
import numpy as np
from matplotlib.patches import Polygon, Rectangle, Circle
from trajsim2d_core.shape_utils import decompose_to_convex_shapes, segment_decreasing_turn
from trajsim2d_core.utils import normal_angle_at_point


## Detect Collision
def detect_collision_AABB(shape_1,shape_2,inflation=0.1):
    """
    @brief Check for collisions between two shapes using bounding boxes/circles
    @param shape_1 is a shape to check for collisions against shape_2
    @param shape_2 is a shape to check for collisions against shape_1
    @return bool: if collision is detected
    """
    collision_flag, overlaps = get_AABB_Overlap(shape_1,shape_2,inflation)
    return collision_flag, overlaps


def detect_collision_distance(shape_1,shape_2,intersecting_area=None,method='GJK'):
    """
    @brief Check for collisions between two shapes using a complex method and measuring distance between
    @param shape_1 is a shape to check for collisions against shape_2
    @param shape_2 is a shape to check for collisions against shape_1
    @param intersecting_area is an optional area to crop the shapes to for collision detection
    @param method is the method to use for collision detection
    @return bool: if collision is detected
    @return bool: distance between shapes
    """


    # detect between a circle v circle
    if isinstance(shape_1,Circle) and isinstance(shape_2,Circle):
        return detect_circle_collision_distance(shape_1,shape_2)
    

    # convert all shapes into np.ndarray of points
    shape_1 = convert_shape_to_numpy(shape_1)
    shape_2 = convert_shape_to_numpy(shape_2)

    # crop np.ndarrays
    if intersecting_area is not None:
        if isinstance(shape_1,np.ndarray):
            shape_1 = crop_shape_to_AABB(shape_1,intersecting_area)
        if isinstance(shape_2,np.ndarray):
            shape_2 = crop_shape_to_AABB(shape_2,intersecting_area)

    
    
    # detect between a np.ndarray v np.ndarray
    # Dictionary mapping method names to functions
    methods = {
        "GJK": gjk_distance,
        "SAT": sat_distance,
        "GRID": grid_distance
    }

    if method not in methods:
        raise ValueError(f"Unknown method '{method}'")
    
    
    if isinstance(shape_1,np.ndarray) and isinstance(shape_2,np.ndarray):
        return methods[method](shape_1,shape_2)    
    else:
        raise NotImplementedError("Distance collision detection not implemented for these shape types.")


def convert_shape_to_numpy(shape):
    """
    @brief Convert a shape to a numpy array of points.
    @param shape is a shape to convert
    @return np.ndarray of shape points
    """
    if isinstance(shape,Circle):
        return circle_to_numpy(shape)
    elif isinstance(shape,Rectangle):
        return shape.get_verts()
    elif isinstance(shape,np.ndarray):
        return shape
    else:
        raise NotImplementedError("Conversion to numpy not implemented for this shape type.")


def detect_circle_collision_distance(circle_1,circle_2):
    """
    @brief Check for collisions between two circles using distance method
    @param circle_1 is a Circle to check for collisions against circle_2
    @param circle_2 is a Circle to check for collisions against circle_1
    @return bool: if collision is detected
    @return bool: distance between circles
    """
    cx1, cy1 = circle_1.center
    r1 = circle_1.radius
    cx2, cy2 = circle_2.center
    r2 = circle_2.radius

    dist_centers = np.sqrt((cx2 - cx1)**2 + (cy2 - cy1)**2)
    distance = dist_centers - (r1 + r2)

    collision = distance < 0.00000000001

    return collision, distance

def detect_shape_bounded(shape,boundary,method='GJK',inflation=1.0,AABB = None,convex_boundary =None):
    """
    @brief checks that the shape is contained in the boundary 
    @param boundary is a shape area to check the shape is contained within
    @param shape is a shape to check it is contained within the boundary
    @return bool: if shape is in bounds
    """
    if boundary is None:
        return True

    if convex_boundary is None:
        convex_boundary = segment_decreasing_turn(boundary)
    
    if shape is None:
        return True
    
    for shape_2 in convex_boundary:
        collision, distance = detect_collision_distance(shape,shape_2,method=method)
        print("Shape out of bounds:",collision, " | Distance:",distance)
        if collision:
            return False

    return True
    

def create_convex_boundary_objects(boundary,inflation=1.0,AABB = None):
    """
    @brief creates a large polygon object surrounding the boundary to use for collision detection
    @param boundary is a shape area to create the object around
    @return list of np.ndarray of boundary object points
    """
    ## Create a large box to create an object surrounding the boundary
    if AABB is None:
        minx, miny, maxx, maxy = get_AABB(boundary,inflation=inflation)
    else:
        minx, miny, maxx, maxy = AABB

    # Get the outer points
    segmented_boundary = segment_decreasing_turn(boundary)

    # Find the tangent at the outpoints
    n = len(segmented_boundary)
    tangent_list = [[None] * n, [None] * n]
    angle = 0.0
    for i, seg in enumerate(segmented_boundary):
        idx = np.where(seg[0] == boundary)[0][0]
        angle = normal_angle_at_point(boundary,idx)
        tangent_list[0][i] = angle
        tangent_list[1][i-1] = angle
    
    
    boundary_objects = []
    # For each segment start to build the object
    for i, seg in enumerate(segmented_boundary):
        
        boundary_object = []
        # Find the point on the AABB that the tangent hits
        aabb_point_1 = aabb_intersection_in_direction(minx, miny, maxx, maxy, seg[0,0],seg[0,1],tangent_list[0][i])
        
        boundary_object = [aabb_point_1]
        
        boundary_object = np.vstack([boundary_object, seg])

        
        # Find the point on the AABB that the tangent hits
        aabb_point_2 = aabb_intersection_in_direction(minx, miny, maxx, maxy, seg[-1,0],seg[-1,1],tangent_list[1][i])
        boundary_object = np.vstack([boundary_object, [aabb_point_2]])

        ##Fill in the boundary points in between
        tol = 1e-9  # tolerance for floating-point comparison
        count = 0
        while not (np.isclose(aabb_point_1[0], aabb_point_2[0], atol=tol) or
           np.isclose(aabb_point_1[1], aabb_point_2[1], atol=tol)):
            count += 1
            if count > 4:
                raise Exception("Infinite loop detected in creating boundary object")

            

            if np.isclose(aabb_point_2[1], miny, atol=tol):
                aabb_point_2 = [minx, miny]
            elif np.isclose(aabb_point_2[0], minx, atol=tol):
                aabb_point_2 = [minx, maxy]
            elif np.isclose(aabb_point_2[1], maxy, atol=tol):
                aabb_point_2 = [maxx, maxy]
            elif np.isclose(aabb_point_2[0], maxx, atol=tol):
                aabb_point_2 = [maxx, miny]
            else:
                print("Warning: aabb_point_2 did not match any boundary conditions")
                break


            boundary_object = np.vstack([boundary_object, [aabb_point_2]])
            

        boundary_object = np.vstack([boundary_object, [aabb_point_1]])
        boundary_objects += [boundary_object]


    
    convex_boundary_objects = []
    for boundary_object in boundary_objects:
        convex_boundary_objects += decompose_to_convex_shapes(boundary_object)

    return convex_boundary_objects


def detect_shapes_bounded(boundary,shapes,inflation=1.0,AABB=None,convex_boundary=None):
    """
    @brief checks that the shapes are contained in the boundary 
    @param boundary is a shape area to check the shapes are contained within
    @param shapes is a list of shapes to check they are contained within the boundary
    @return bool: if shapes_1 are in bounds
    """
    if boundary is None:
        return True
    
    if shapes is None:
        return True
    
    for shape in shapes:
        if not detect_shape_bounded(shape,boundary,inflation=inflation,AABB=AABB,convex_boundary=convex_boundary):
            return False

    return True

def detect_any_collisions(shapes_1,shapes_2,max_distance=0.2,method='GJK'):
    """
    @brief Iterates through all shapes and checks for collisions between them.

    @param shapes_1 List of shapes representing the first set of shapes to check for collisions.
    @param shapes_2 List of shapes representing the second set of shapes to check against shapes_1.
    @param max_distance Float representing the maximum distance to obtain a measurement of how far objects are away from each other.
    @return bool True if any collision is detected, False otherwise.
    @return collision_distances List of tuples (shape_1, shape_2, distance) representing which shapes are colliding and the distance between them.
    """

    # Create a list of potentially intersecting shapes and their potentially intersecting areas using bounding boxes
    any_collisions_flag, pot_collision_shapes = detect_any_collisions_AABB(shapes_1,shapes_2,inflation=max_distance)

    if not any_collisions_flag:
        return False,[]
    
    # For all potentially intersecting shapes check for more in detail collisions:
    any_collisions_flag, collision_distances = detect_any_collisions_distance(pot_collision_shapes,method)

    return any_collisions_flag, collision_distances

def detect_any_collisions_AABB(shapes_1,shapes_2,inflation=0.1):
    """
    @brief Checks for collisions between two sets of shapes using AABB (Axis-Aligned Bounding Box).

    @param shapes_1 List of shapes to check for collisions.
    @param shapes_2 List of shapes to check against shapes_1.
    @param inflation Optional float. Margin to inflate the bounding boxes when checking for collisions. Default is 0.1.

    @return bool True if no collisions are detected, False otherwise.
    @return list potential_collisions List of tuples (shape_1, shape_2, intersecting_area) for all detected collisions.
    """

    potential_collisions = []

    for i, shape_1 in enumerate(shapes_1):
        for shape_2 in shapes_1[i+1:]+shapes_2:
            if shape_1 is shape_2:
                continue

            collision, intesecting_area = detect_collision_AABB(shape_1,shape_2,inflation)
            if collision:
                potential_collisions.append((shape_1,shape_2,intesecting_area))
            
    return bool(potential_collisions), potential_collisions
            
def detect_any_collisions_distance(shapes,method='GJK'):
    """
    @brief Checks for collisions or distances between shape pairs using a distance-based algorithm (placeholder).

    @param shapes List of tuples (shape_1, shape_2, intersecting_area) representing potential collisions.

    @return bool True if any collision is detected, False otherwise.
    @return list distances List of tuples (shape_1, shape_2, distance) giving the computed distance or overlap between each shape pair.
    """

    collision_flag = False
    distances = []

    for shape_1, shape_2, intersecting_area in shapes:
            
        collision, distance = detect_collision_distance(shape_1,shape_2,intersecting_area,method)
        
        if collision:
            collision_flag = True
        distances.append((shape_1,shape_2,distance))
            
    return collision_flag, distances

def detect_any_collisions_bounded(boundary,shapes_1,shapes_2):
    """
    @brief checks that the shapes_1 are within the boundary and theer are no collisions
    internally between shapes_1 and between shapes_1 and shapes_2
    @param boundary is a shape area to check the shapes_1 are contained within
    @param shapes_1 is a list of shapes to check for collisions of inclusive
    @param shapes_2 is a list of shapes to check for collisions of against shapes_1
    @return bool: if collision is detected or shapes_1 out of bounds
    """

    return detect_any_collisions(shapes_1,shapes_2) and not detect_shapes_bounded(boundary,shapes_1,convex_boundary=create_convex_boundary_objects(boundary))



def get_AABB(shape, inflation=0.0):
    """
    @brief returns min and max x and y coordinates for the shape

    @param shape Rectangle, Circle, or np.ndarray shape to get bounding values for
    @param inflation Float amount to inflate the min and max values
    @return Tuple (minx, miny, maxx, maxy)
    """

    # LOGIC
    if isinstance(shape,Circle):
        minx,miny,maxx,maxy = get_AABB_Circle(shape)
    elif isinstance(shape,Rectangle):
        minx,miny,maxx,maxy = get_AABB_Rectangle(shape)
    elif isinstance(shape,np.ndarray):
        minx,miny,maxx,maxy = get_AABB_Array(shape)
    else:
        minx,miny,maxx,maxy = 0.0,0.0,0.0,0.0

    return (minx-inflation,miny-inflation,maxx+inflation,maxy+inflation)

def get_AABB_Circle(circle):
    """
    @brief Returns the axis-aligned bounding box (AABB) of a matplotlib Circle.

    @param circle Circle object.
    @return (minx, miny, maxx, maxy). Returns zeros if not a Circle.
    """

    if not isinstance(circle,Circle):
        return 0.0,0.0,0.0,0.0

    cx, cy = circle.center
    r = circle.radius

    minx = cx - r
    maxx = cx + r
    miny = cy - r
    maxy = cy + r
    return minx, miny, maxx, maxy

def get_AABB_Rectangle(rectangle):
    """
    @brief Returns the axis-aligned bounding box (AABB) of a matplotlib Rectangle.

    Supports rotated rectangles by transforming corner points.

    @param rectangle Rectangle object.
    @return (minx, miny, maxx, maxy). Returns zeros if not a Rectangle.
    """
    if not isinstance(rectangle,Rectangle):
        return 0.0,0.0,0.0,0.0
    
    # Extract rectangle properties
    x = rectangle.get_x()
    y = rectangle.get_y()
    w = rectangle.get_width()
    h = rectangle.get_height()
    angle = rectangle.angle  # degrees, CCW, about (x, y)

    # Unrotated corner points
    corners = np.array([
        [x,     y    ],  # bottom-left (rotation anchor)
        [x+w,   y    ],  # bottom-right
        [x+w,   y+h  ],  # top-right
        [x,     y+h  ]   # top-left
    ])

    # If not rotated, fast path
    if angle == 0:
        xs = corners[:,0]
        ys = corners[:,1]
        return xs.min(), ys.min(), xs.max(), ys.max()

    # Rotation matrix
    theta = np.deg2rad(angle)
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

    # Rotate around (x, y)
    rotated = (R @ (corners - np.array([x, y])).T).T + np.array([x, y])

    xs = rotated[:,0]
    ys = rotated[:,1]

    return xs.min(), ys.min(), xs.max(), ys.max()

def get_AABB_Array(array):
    """
    @brief Returns the axis-aligned bounding box (AABB) of a 2D point array.

    @param array Nx2 numpy array of (x, y) points.
    @return (minx, miny, maxx, maxy). Returns zeros if not an array.
    """
    if not isinstance(array,np.ndarray):
        return 0.0,0.0,0.0,0.0

    xs = array[:,0]
    ys = array[:,1]

    minx = xs.min()
    maxx = xs.max()
    miny = ys.min()
    maxy = ys.max()

    return minx, miny, maxx, maxy

def get_AABB_Overlap(shape_1,shape_2, inflation=0.0):
    """
    @brief determines the overlap between two shapes
    
    @param shape_1 Rectangle, Circle, np.ndarray, or Tuple  (minx, miny, maxx, maxy)
    @param shape_2 Rectangle, Circle, np.ndarray, or Tuple  (minx, miny, maxx, maxy)
    @return bool if there is any overlap
    @return Tuple (minx, miny, maxx, maxy) overlap bounds
    """
    if not isinstance(shape_1,tuple):
        shape_1 = get_AABB(shape_1,inflation)

    if not isinstance(shape_2,tuple):
        shape_2 = get_AABB(shape_2,inflation)
    
    overlap = False
    overlap_bounds = [0.0,0.0,0.0,0.0]

    minx1, miny1, maxx1, maxy1 = np.array(shape_1,dtype=float)
    minx2, miny2, maxx2, maxy2 = np.array(shape_2,dtype=float)

    overlap_x = maxx1 > minx2 and maxx2 > minx1
    overlap_y = maxy1 > miny2 and maxy2 > miny1
    overlap = overlap_x and overlap_y

    if not overlap:
        return False, (0.0, 0.0, 0.0, 0.0)

    overlap_bounds = (
        max(minx1, minx2),
        max(miny1, miny2),
        min(maxx1, maxx2),
        min(maxy1, maxy2)
    )

    return True, overlap_bounds



def crop_shape_to_AABB(shape, boundary):
    """
    @brief Uses the Sutherland-Hodgeman Polygon Clipping Algorithm to crop a polygonal shape to a polygonal boundary.
    
    This function removes points outside the boundary, adds points where
    the shape intersects the boundary, and includes boundary points that
    lie inside the shape. Returns a NumPy array of points representing
    the cropped polygon.
    
    @param shape np.ndarray of shape (N,2): points of the polygonal shape
    @param boundary Tuple (minx, miny, maxx, maxy)
    
    @return np.ndarray of points representing the cropped polygon
    """
    if not isinstance(shape, np.ndarray):
        raise TypeError("Shape must be a numpy ndarray")
    if not (isinstance(boundary, (tuple, list)) and len(boundary) == 4):
        raise TypeError("Boundary must be a tuple (minx, miny, maxx, maxy)")

    
    cropped_shape = suthHodgClip (shape,len(shape), [
            [boundary[0], boundary[3]],  # top-left
            [boundary[2], boundary[3]],  # top-right
            [boundary[2], boundary[1]],  # bottom-right
            [boundary[0], boundary[1]]   # bottom-left
        ],4)


    return cropped_shape



def circle_to_numpy(circle, num_points=100):
    """
    @brief Convert a matplotlib Circle to a NumPy array of points approximating the circle.
    
    @param circle matplotlib.patches.Circle object
    @param num_points int Number of points to approximate the circle (default: 100)
    
    @return np.ndarray of shape (num_points, 2) representing the circle's points
    """
    if not isinstance(circle, Circle):
        raise TypeError("Input must be a matplotlib.patches.Circle")
    
    center = circle.center
    radius = circle.radius

    theta = np.linspace(0, 2*np.pi, num_points, endpoint=False)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)

    return np.stack((x, y), axis=-1)


def point_in_polygon(point, polygon):
    """
    @brief Check if a point is inside a polygon using the ray-casting algorithm.
    @param point is a np.ndarray point to check
    @param polygon is a np.ndarray of polygon points
    @return bool: if point is inside polygon
    """
    x, y = point
    n = len(polygon)
    inside = False

    for i in range(n):
        j = (i + 1) % n
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        # Check if the ray intersects the edge
        intersect = ((yi > y) != (yj > y)) and \
                    (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
        if intersect:
            inside = not inside

    return inside
    

#######################################################################
#                                                                     #
#                  GJK DISTANCE BETWEEN OBSTACLES                     #
#                                                                     #
#  This section implements the Gilbert-Johnson-Keerthi (GJK)          #
#  algorithm to compute the minimum distance between two convex       #
#  shapes or obstacles.                                               #
#                                                                     #
#######################################################################

def gjk_distance(shape_1,shape_2):
    """
    @brief GJK distance algorithm between two np.ndarray shapes
    @param shape_1 is a np.ndarray shape to check for collisions against shape_2
    @param shape_2 is a np.ndarray shape to check for collisions against shape_1
    @return bool: if collision is detected
    @return float: distance between shapes
    """
    smallest_distance = float('inf')
    vector_polygon = []

    for p1 in shape_1:
        for p2 in shape_2:
            vector = p2 - p1
            dist = np.linalg.norm(vector)
            if dist < smallest_distance:
                smallest_distance = dist
            vector_polygon.append(vector)
    
    if point_in_polygon(np.array([0.0,0.0]),np.array(vector_polygon)):
        return True, 0.0
    
    if smallest_distance > 0.03:
        return False, smallest_distance
    
    return False, smallest_distance



def sat_distance(shape_1,shape_2):
    """
    @brief SAT distance algorithm between two np.ndarray shapes
    @param shape_1 is a np.ndarray shape to check for collisions against shape_2
    @param shape_2 is a np.ndarray shape to check for collisions against shape_1
    @return bool: if collision is detected
    @return float: distance between shapes
    """
    # Placeholder implementation
    return False, 0.0

def grid_distance(shape_1,shape_2):
    """
    @brief GRID distance algorithm between two np.ndarray shapes
    @param shape_1 is a np.ndarray shape to check for collisions against shape_2
    @param shape_2 is a np.ndarray shape to check for collisions against shape_1
    @return bool: if collision is detected
    @return float: distance between shapes
    """
    # Placeholder implementation
    return False, 0.0

#######################################################################
#                                                                     #
#      SUTHERLAND–HODGMAN POLYGON CLIPPING ALGORITHM                  #
#                                                                     #
#  This section implements the Sutherland–Hodgman polygon clipping    #
#  algorithm, adapted from the GeeksforGeeks tutorial.               #
#                                                                     #
#  It clips a subject polygon against a convex clipping polygon       #
#  (e.g., an AABB), producing a new polygon consisting of only the    #
#  points inside the clipping area.                                   #
#                                                                     #
#######################################################################


# Defining maximum number of points in polygon
MAX_POINTS = 300

# Function to return x-value of point of intersection of two lines
def x_intersect(x1, y1, x2, y2, x3, y3, x4, y4):
    num = (x1*y2 - y1*x2) * (x3-x4) - (x1-x2) * (x3*y4 - y3*x4)
    den = (x1-x2) * (y3-y4) - (y1-y2) * (x3-x4)
    return num/den

# Function to return y-value of point of intersection of two lines
def y_intersect(x1, y1, x2, y2, x3, y3, x4, y4):
    num = (x1*y2 - y1*x2) * (y3-y4) - (y1-y2) * (x3*y4 - y3*x4)
    den = (x1-x2) * (y3-y4) - (y1-y2) * (x3-x4)
    return num/den

# Function to clip all the edges w.r.t one clip edge of clipping area
def clip(poly_points, poly_size, x1, y1, x2, y2):
    new_points = np.zeros((MAX_POINTS, 2), dtype=float)
    new_poly_size = 0

    # (ix,iy),(kx,ky) are the co-ordinate values of the points
    for i in range(poly_size):
        # i and k form a line in polygon
        k = (i+1) % poly_size
        ix, iy = poly_points[i]
        kx, ky = poly_points[k]

        # Calculating position of first point w.r.t. clipper line
        i_pos = (x2-x1) * (iy-y1) - (y2-y1) * (ix-x1)
        # Calculating position of second point w.r.t. clipper line
        k_pos = (x2-x1) * (ky-y1) - (y2-y1) * (kx-x1)

        # Case 1 : When both points are inside
        if i_pos < 0 and k_pos < 0:
            # Only second point is added
            new_points[new_poly_size] = [kx, ky]
            new_poly_size += 1

        # Case 2: When only first point is outside
        elif i_pos >= 0 and k_pos < 0:
            # Point of intersection with edge and the second point is added
            new_points[new_poly_size] = [x_intersect(x1, y1, x2, y2, ix, iy, kx, ky),
                                         y_intersect(x1, y1, x2, y2, ix, iy, kx, ky)]
            new_poly_size += 1
            new_points[new_poly_size] = [kx, ky]
            new_poly_size += 1

        # Case 3: When only second point is outside
        elif i_pos < 0 and k_pos >= 0:
            # Only point of intersection with edge is added
            new_points[new_poly_size] = [x_intersect(x1, y1, x2, y2, ix, iy, kx, ky),
                                         y_intersect(x1, y1, x2, y2, ix, iy, kx, ky)]
            new_poly_size += 1

        # Case 4: When both points are outside
        else:
            pass  # No points are added, but we add a pass statement to avoid the IndentationError

    # Copying new points into a separate array and changing the no. of vertices
    clipped_poly_points = np.zeros((new_poly_size, 2), dtype=float)
    for i in range(new_poly_size):
        clipped_poly_points[i] = new_points[i]

    return clipped_poly_points, new_poly_size

# Function to implement Sutherland–Hodgman algorithm
def suthHodgClip(poly_points, poly_size, clipper_points, clipper_size):
    # i and k are two consecutive indexes
    for i in range(clipper_size):
        k = (i+1) % clipper_size

        # We pass the current array of vertices, it's size and the end points of the selected clipper line
        poly_points, poly_size = clip(poly_points, poly_size, clipper_points[i][0],
                                      clipper_points[i][1], clipper_points[k][0],
                                      clipper_points[k][1])


    return poly_points[:poly_size]


def aabb_intersection_in_direction(minx, miny, maxx, maxy, x0, y0, theta, tol=1e-9):
    """
    @brief Computes the intersection point of a line with an axis-aligned bounding box (AABB)
           in the specified direction from a given point.

    @param minx Minimum x-coordinate of the box
    @param miny Minimum y-coordinate of the box
    @param maxx Maximum x-coordinate of the box
    @param maxy Maximum y-coordinate of the box
    @param x0   x-coordinate of a point on the line
    @param y0   y-coordinate of a point on the line
    @param theta Angle of the line in radians (0 along positive x-axis)
    @param tol   Floating-point tolerance for comparisons (default: 1e-9)

    @return np.ndarray or None
           A numpy array [x, y] representing the intersection point in the
           forward direction along the line. Returns None if the line does
           not intersect the box.
    """
    dx = np.cos(theta)
    dy = np.sin(theta)
    dir_vec = np.array([dx, dy])

    # List to store valid intersections
    intersections = []

    # Check intersections with vertical edges (x = minx, maxx)
    if abs(dx) > tol:
        for x_edge in [minx, maxx]:
            t = (x_edge - x0) / dx
            y = y0 + t * dy
            if miny - tol <= y <= maxy + tol:
                intersections.append((t, x_edge, y))

    # Check intersections with horizontal edges (y = miny, maxy)
    if abs(dy) > tol:
        for y_edge in [miny, maxy]:
            t = (y_edge - y0) / dy
            x = x0 + t * dx
            if minx - tol <= x <= maxx + tol:
                intersections.append((t, x, y_edge))

    if not intersections:
        return None  # line does not intersect the box

    # Select the intersection in the forward direction along the line
    best = max(intersections, key=lambda p: np.dot([p[1]-x0, p[2]-y0], dir_vec))
    _, x, y = best
    return np.array([x, y])