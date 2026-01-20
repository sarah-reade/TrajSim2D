from matplotlib.patches import Rectangle, Circle
import numpy as np

## Function for breaking down concave into convex shapes
def decompose_to_convex_shapes(shape, max_iterations=10,segment_method='increasing_turn'):
    """
    @brief Decomposes a concave shape into convex shapes.
    @param shape Nx2 np.ndarray representing the concave shape.
    @return List of Nx2 np.ndarrays, each representing a convex part of the shape.
    """
    
    new_polygon = []
    segmented_shape_list = []
    counter = 0

    
    if segment_method == 'decreasing_turn':
        segment_shape_by_turn = segment_decreasing_turn
    else:
        segment_shape_by_turn = segment_increasing_turn

    segmented_shapes = segment_shape_by_turn(shape)

    while len(segmented_shapes) > 1 and counter < max_iterations:
        
        new_polygon = []
        segmented_shape_list += segmented_shapes.copy()
        #print("segmented_shapes: ",segmented_shapes)
        
                
        for seg in segmented_shapes:
            if len(seg) > 0:
                new_polygon.append(seg[0])
        
        new_polygon.append(segmented_shapes[0][0])
        
        segmented_shapes = segment_shape_by_turn(new_polygon)
        counter += 1

    if new_polygon:
        segmented_shape_list.append(np.vstack(new_polygon.copy()))
    return segmented_shape_list



def make_complex_concave_shape(n=300, num_pits=5, seed=None):
    """
    Create a naturally wobbly, irregular concave 2D shape.
    """
    if seed is not None:
        np.random.seed(seed)

    t = np.linspace(0, 2*np.pi, n, endpoint=False)

    # --- Base radius: sum of sine waves with random amplitude, phase, and frequency ---
    r = 5 * np.ones_like(t)
    num_waves = 5
    for k in range(1, num_waves+1):
        amplitude = 0.3 + 0.5*np.random.rand()       # random amplitude between 0.3-0.8
        phase = 2*np.pi*np.random.rand()             # random phase
        freq = k + np.random.rand()                  # slightly irregular frequency
        r += amplitude * np.sin(freq*t + phase)

    # --- Add random concave pits ---
    for _ in range(num_pits):
        center = 2*np.pi*np.random.rand() - np.pi     # random angular position
        width = 0.3 + 0.5*np.random.rand()            # random angular half-width
        depth = 1.5 + 1.0*np.random.rand()            # random radial depth
        dt = np.angle(np.exp(1j*(t - center)))
        mask = np.abs(dt) < width
        strength = 1 - (np.abs(dt[mask]) / width)
        r[mask] -= strength * depth

    # Ensure radius remains positive
    r = np.clip(r, 0.1, None)

    # Convert to Cartesian coordinates
    x = r * np.cos(t)
    y = r * np.sin(t)
    pts = np.stack([x, y], axis=1)

    # --- Close the shape by repeating the first point ---
    pts = np.vstack([pts, pts[0]])

    return pts


def segment_by_max_gradient(points, angle_threshold_deg=90):
    """
    Splits points into segments based on gradient deviations.
    Each segment ends at the point of maximum deviation, 
    or at a local minima between points of large deviation.

    Args:
        points : Nx2 array of ordered 2D points
        angle_threshold_deg : threshold in degrees to cut segment

    Returns:
        segments : list of arrays
            Each array is a segment of points
        all_diffs : np.ndarray
            Angular deviation (deg) from segment's initial direction per point
    """
    pts = np.asarray(points)
    N = len(pts)
    if N < 3:
        return [pts], np.zeros(len(pts))

    # Direction vectors and normalized
    dirs = pts[1:] - pts[:-1]
    norms = np.linalg.norm(dirs, axis=1)
    norms[norms == 0] = 1e-12
    dirs = dirs / norms[:, None]

    # Convert to angles
    dir_angles = np.degrees(np.arctan2(dirs[:,1], dirs[:,0]))

    segments = []
    all_diffs = np.zeros(len(pts))

    start_idx = 0
    initial_angle = dir_angles[0]

    max_diff = 0
    max_diff_point = start_idx
    last_diff = 0

    for i in range(1, len(dirs)):
        diff = dir_angles[i] - initial_angle
        diff = (diff + 180) % 360 - 180
        all_diffs[i] = diff

        # Track max difference
        if abs(diff) > abs(max_diff):
            max_diff = diff
            max_diff_point = i

        # Check for local minima in diff magnitude
        # i.e., diff magnitude decreasing after a peak
        if abs(diff) < abs(last_diff) and abs(last_diff) > angle_threshold_deg:
            # Cut at the previous peak (local maximum)
            segments.append(pts[start_idx:max_diff_point+1])
            start_idx = max_diff_point + 1

            if start_idx < len(dirs):
                initial_angle = dir_angles[start_idx]

            # Reset tracking
            last_diff = 0
            max_diff = 0
            max_diff_point = start_idx

        last_diff += diff

        # Also check absolute threshold
        if abs(diff) > angle_threshold_deg:
            segments.append(pts[start_idx:max_diff_point+1])
            start_idx = max_diff_point + 1

            if start_idx < len(dirs):
                initial_angle = dir_angles[start_idx]

            max_diff = 0
            max_diff_point = start_idx
            last_diff = 0

    # Add last segment
    if start_idx < len(pts):
        segments.append(pts[start_idx:])

    return segments, all_diffs


def turning_angles(points):
    """
    Compute the change in gradient (turning angle) at each vertex of a polygon.
    
    Parameters:
        points: Nx2 array of polygon points in order.
        
    Returns:
        angles: N-array of turning angles in radians.
    """
    points = np.array(points)
    n = len(points)
    angles = np.zeros(n)
    
    for i in range(n):
        prev = points[i-1]
        curr = points[i]
        next = points[(i+1)%n]
        
        # Vectors
        v1 = curr - prev
        v2 = next - curr
        
        # Angles
        theta1 = np.arctan2(v1[1], v1[0])
        theta2 = np.arctan2(v2[1], v2[0])
        
        # Turning angle
        delta_theta = theta2 - theta1
        
        # Normalize to [-pi, pi]
        delta_theta = (delta_theta + np.pi) % (2*np.pi) - np.pi
        
        angles[i] = delta_theta
    
    return angles

def turning_angles_derivative(points):
    """
    Compute the discrete derivative of turning angles along a polygon.
    
    Parameters:
        points: Nx2 array of polygon points in order.
        
    Returns:
        derivative: (N,) array of discrete derivatives of turning angles.
    """
    # First compute turning angles
    angles = turning_angles(points)
    
    # Discrete derivative (difference between consecutive angles)
    # We'll use np.diff with wrap-around for closed polygon
    derivative = np.zeros_like(angles)
    derivative[:-1] = np.diff(angles)
    derivative[-1] = angles[0] - angles[-1]  # wrap-around last->first
    
    # Optional: normalize to [-pi, pi] like before
    derivative = (derivative + np.pi) % (2*np.pi) - np.pi
    
    return derivative

def segment_by_derivative_zero_cross(points):
    """
    Segment polygon points whenever the derivative of turning angles crosses zero.
    
    Parameters:
        points: Nx2 array of polygon points in order.
        
    Returns:
        segments: list of arrays, each containing consecutive points in a segment.
    """
    points = np.array(points)
    
    # Compute turning angles and derivatives
    angles = turning_angles(points)
    derivative = np.zeros_like(angles)
    derivative[:-1] = np.diff(angles)
    derivative[-1] = angles[0] - angles[-1]  # wrap-around
    derivative = (derivative + np.pi) % (2*np.pi) - np.pi
    
    # Identify zero-crossings: sign change
    sign_changes = np.where(np.diff(np.sign(derivative)) != 0)[0] + 1
    
    # Split points at zero-crossings
    segments = []
    start_idx = 0
    for idx in sign_changes:
        segments.append(points[start_idx:idx])
        start_idx = idx
    segments.append(points[start_idx:])  # last segment
    
    return segments


def segment_decreasing_turn(points):
    """
    Segment polygon points where the derivative of turning angles is negative
    (i.e., turning angles are decreasing).
    
    Parameters:
        points: Nx2 array of polygon points in order.
        
    Returns:
        segments: list of arrays, each containing consecutive points where derivative > 0.
    """
    points = np.array(points)
    n = len(points)
    
    # Compute turning angles
    angles = turning_angles(points)
    
    # Compute discrete derivative
    derivative = np.zeros(n)
    derivative[:-1] = np.diff(angles)
    derivative[-1] = angles[0] - angles[-1]  # wrap-around
    derivative = (derivative + np.pi) % (2*np.pi) - np.pi
    
    # Boolean mask: True where derivative >= 0 and previous point was > 0
    decreasing_mask = (derivative <= 0) & (np.roll(derivative, 1) > 0)
    
    # Collect consecutive segments, include the next point as well
    segments = []
    current_segment = []
    partial_first_segment = []
    
    # Iterate through points
    for i in range(n):
        # Add point to current segment
        current_segment.append(points[i])

        # If this is the point where derivative crosses 0 with an increasing value
        if decreasing_mask[i]:
            # If this is the first segment:
            if not partial_first_segment:
                # Store current_segment in partial_first_segment
                partial_first_segment = current_segment.copy()
                # Reset current_segment
                current_segment = [points[i]]
            
            # Else append to segments
            else:
                if current_segment:
                    segments.append(np.array(current_segment))
                # Reset current_segment
                current_segment = [points[i]]
            

    segments.append(np.array(current_segment + partial_first_segment))
    return segments

def segment_increasing_turn(points):
    """
    Segment polygon points where the derivative of turning angles is positive
    (i.e., turning angles are increasing).
    
    Parameters:
        points: Nx2 array of polygon points in order.
        
    Returns:
        segments: list of arrays, each containing consecutive points where derivative > 0.
    """
    points = np.array(points)
    n = len(points)
    
    # Compute turning angles
    angles = turning_angles(points)
    
    # Compute discrete derivative
    derivative = np.zeros(n)
    derivative[:-1] = np.diff(angles)
    derivative[-1] = angles[0] - angles[-1]  # wrap-around
    derivative = (derivative + np.pi) % (2*np.pi) - np.pi
    
    # Boolean mask: True where derivative >= 0 and previous point was < 0
    increasing_mask = (derivative >= 0) & (np.roll(derivative, 1) < 0)
    
    # Collect consecutive segments, include the next point as well
    segments = []
    current_segment = []
    partial_first_segment = []
    
    # Iterate through points
    for i in range(n):
        # Add point to current segment
        current_segment.append(points[i])

        # If this is the point where derivative crosses 0 with an increasing value
        if increasing_mask[i]:
            # If this is the first segment:
            if not partial_first_segment:
                # Store current_segment in partial_first_segment
                partial_first_segment = current_segment.copy()
                # Reset current_segment
                current_segment = [points[i]]
            
            # Else append to segments
            else:
                if current_segment:
                    segments.append(np.array(current_segment))
                # Reset current_segment
                current_segment = [points[i]]
            

    segments.append(np.array(current_segment + partial_first_segment))
    return segments

def split_array_at_indices(boundary, idx_a, idx_b):
    """
    Returns the two paths between idx_a and idx_b on a closed polygon.
    """
    if idx_a <= idx_b:
        path_a_to_b = boundary[idx_a:idx_b + 1]
        path_b_to_a = np.vstack((boundary[idx_b:], boundary[:idx_a + 1]))
    else:
        path_a_to_b = np.vstack((boundary[idx_a:], boundary[:idx_b + 1]))
        path_b_to_a = boundary[idx_b:idx_a + 1]

    return path_a_to_b, path_b_to_a

def get_shape_size(shape):
    '''
    @brief Returns a measure of the size of the shape.
    @param shape: np.ndarray or shape object
    @return size: float representing size (e.g., number of points, area, etc
    '''
    if isinstance(shape, np.ndarray):
        return len(shape)  # number of points
    elif hasattr(shape, "radius"):  # Circle
        return 100
    elif hasattr(shape, "width") and hasattr(shape, "height"):  # Rectangle
        return 4
    else:
        return 0  # fallback if unknown type
    
    
def is_convex(a, b, c):
    return np.cross(b - a, c - b) >= 0

def point_in_triangle(p, a, b, c):
    v0 = c - a
    v1 = b - a
    v2 = p - a

    dot00 = np.dot(v0, v0)
    dot01 = np.dot(v0, v1)
    dot02 = np.dot(v0, v2)
    dot11 = np.dot(v1, v1)
    dot12 = np.dot(v1, v2)

    inv = 1.0 / (dot00 * dot11 - dot01 * dot01)
    u = (dot11 * dot02 - dot01 * dot12) * inv
    v = (dot00 * dot12 - dot01 * dot02) * inv

    return (u >= 0) and (v >= 0) and (u + v <= 1)
    
def triangulate_simple_polygon(poly):
    """
    Ear clipping triangulation.
    Returns list of triangles (each (3,2) np.ndarray)
    """
    poly = poly.tolist()
    triangles = []

    while len(poly) > 3:
        n = len(poly)
        ear_found = False

        for i in range(n):
            prev = np.array(poly[(i - 1) % n])
            curr = np.array(poly[i])
            next = np.array(poly[(i + 1) % n])

            if not is_convex(prev, curr, next):
                continue

            if any(
                point_in_triangle(np.array(p), prev, curr, next)
                for j, p in enumerate(poly)
                if j not in [(i - 1) % n, i, (i + 1) % n]
            ):
                continue

            triangles.append(np.array([prev, curr, next]))
            del poly[i]
            ear_found = True
            break

        if not ear_found:
            raise RuntimeError("Triangulation failed")

    triangles.append(np.array(poly))
    return triangles


def shape_in_tuple_list(shapes, tuple_list):
    """
    @brief Checks if given shapes is present in a list of tuples.

    Supports matplotlib.patches.Rectangle, matplotlib.patches.Circle, and numpy.ndarray.

    @param shape The shape to check (Rectangle, Circle, or np.ndarray).
    @param tuple_list A list of tuples, each containing two shapes and a distance.
    @return True if the shape exists in any tuple in the list; False otherwise.
    """
    for tup in tuple_list:
        shapes_in_tup = tup[:2]  # first two elements are shapes
        for candidate_shape in shapes_in_tup:
            for shape in shapes:
                # print("Comparing ", shape, " with ", candidate_shape)
                if type(shape) == type(candidate_shape):
                    if isinstance(shape, Rectangle):
                        if (shape.get_xy() == candidate_shape.get_xy() and
                            shape.get_width() == candidate_shape.get_width() and
                            shape.get_height() == candidate_shape.get_height() and
                            shape.get_angle() == candidate_shape.get_angle()):
                            return True
                    elif isinstance(shape, Circle):
                        if (shape.center == candidate_shape.center and
                            shape.get_radius() == candidate_shape.get_radius()):
                            return True
                    elif isinstance(shape, np.ndarray):
                        if np.array_equal(shape, candidate_shape):
                            return True
    return False