###############################################################################
## @file environment.py
## @brief 
##
## This file is part of the TrajSim2D project, a 2D planar manipulator simulator
## for trajectory planning, collision testing, and environment visualization.
## 
## Module responsibilities:
## - <List main purpose of this file, e.g., "Forward kinematics and Jacobian calculations">
## - <Optional: physics, dynamics, or utility functions>
##
## Author: Sarah Reade
## Email: 28378329@students.lincoln.ac.uk
## Date: 2025-10-23
## Version: 0.0.1
##
## License: MIT
##
## Usage:
## >>> from trajsim2d_core.manip import PlanarManipulator
## >>> arm = PlanarManipulator(num_joints=3, link_lengths=[1.0, 1.0, 0.5])
## >>> q = [0.0, 1.0, 0.5]
## >>> pos = arm.forward_kinematics(q)
###############################################################################

# Imports
import numpy as np
from scipy.ndimage import gaussian_filter1d  # smoother hills
from trajsim2d_core.utils import generate_random_number,generate_random_int,generate_random_int_array,get_random_array_from_list,get_random_coordinate, normal_angle_at_point,tangent_angle_at_point
from trajsim2d_core.collision import detect_shapes_bounded

# Initialise environment
global BORDER_SIZE 
BORDER_SIZE = 5
global BORDER_CENTRE 
BORDER_CENTRE = BORDER_SIZE/2

## Generate random border
def generate_random_border(border_size=BORDER_SIZE, smoothness=1, num_points=200):
    """
    @brief Generate a smooth, random circular border with periodic bumps.
    @param border_size Approximate diameter of the border; border is centered at border_size/2.
    @param smoothness Float in [0,1]; 1 = perfect circle, 0 = maximum bumps.
    @param num_points Number of points along the border.
    @return Nx2 np.ndarray of (x, y) coordinates defining the border.
    """
    
    global BORDER_SIZE 
    BORDER_SIZE = border_size
    global BORDER_CENTRE
    BORDER_CENTRE = border_size/2
    return generate_random_circle(border_size,smoothness,num_points)


def generate_random_objects(num_points=60,object_size=None,object_max=None,smoothness=None,num_objs=None,border_size=None,attempts=100,border=None):
    """
    @brief Generate a smooth, random circular object with periodic bumps.
    @param num_points Number of points per object.
    @param object_size Fixed size for each object (optional).
    @param object_max Maximum size allowed for objects (optional).
    @param smoothness Float [0,1] controlling bump smoothness (optional).
    @param num_objects Number of objects to generate (optional).
    @return List of Nx2 np.ndarrays, each representing a single object's coordinates.
    """
    objs = []
    
    global BORDER_SIZE
    if border_size == None:
        border_size = BORDER_SIZE

    if object_size != None:
        object_max = object_size
    

    elif object_max == None:
        object_max = generate_random_number(0,border_size/4)
    
    
    #print("object_size: ",object_size)
    #print("object_max: ",object_max)    

    if num_objs == None:
        num_objs = generate_random_int(1,(border_size/object_max))
    
    #print("num_objs: ",num_objs)

    for i in range(0,num_objs):
        if smoothness == None:
            smoothness = generate_random_number(0,1)

        #print("smoothness: ",smoothness)

        if object_size == None:
            object_i_size = generate_random_number(0.1,object_max)
        else:
            object_i_size = object_size

        centre = [generate_random_number(0.1,border_size),generate_random_number(0.1,border_size)]
        circle = generate_random_circle(circle_size=object_i_size,smoothness=smoothness,num_points=num_points,circle_centre=centre)
        
        ## check that the circle is bounded
        counter = 0
        if border is not None:
            while not detect_shapes_bounded(border,[circle]) and counter < attempts/num_objs:
                centre = [generate_random_number(0.1,border_size),generate_random_number(0.1,border_size)]
                circle = generate_random_circle(circle_size=object_i_size,smoothness=smoothness,num_points=num_points,circle_centre=centre)
                counter += 1
                
        
        objs.append(circle)

    #print("objs: ",objs)
    return objs

    

## Generate a bumby circle 
def generate_random_circle(circle_size=1, smoothness=1, num_points=200,circle_centre=None):
    """
    @brief Generate a smooth, random circle with periodic bumps.
    @param circle_size Diameter of the circle.
    @param smoothness Float in [0,1]; 1 = perfect circle, 0 = maximum bumps.
    @param num_points Number of points along the circle perimeter.
    @param circle_centre Optional 2-element list or array specifying the circle center. Defaults to [circle_size/2, circle_size/2].
    @return Nx2 np.ndarray of (x, y) coordinates defining the circle.
    """

    if circle_centre is None:
        circle_centre = [circle_size / 2,circle_size / 2]

    radius = circle_size / 2
    
    angles = np.linspace(0, 2*np.pi, num_points, endpoint=False)
    
    # Random bumps
    max_bump = radius * (1 - smoothness)
    bumps = np.abs(generate_random_int_array(-1,1,num_points)) * max_bump
    #print(bumps)
    
    # Circular Gaussian smoothing (wrap mode)
    sigma = max(min(num_points / 30, num_points - 1), 1e-6)
    #print("bumps: ",bumps)
    #print("sigma: ",sigma)
    #print("num_points",num_points)
    bumps = gaussian_filter1d(bumps, sigma, mode='wrap')
    
    # Final radius
    r = radius + bumps
    
    
    # Convert polar to Cartesian
    x = circle_centre[0] + r * np.cos(angles)
    y = circle_centre[1] + r * np.sin(angles)
    
    return np.column_stack((x, y))

## Generate Random edge Point
def generate_random_edge_point(border=None,objs=None):
    """
    @brief Generate a random point along a given border or object for attachment.
    @details Also returns the clockwise angle (from the y-axis) that is tangential to the object.
             For borders, the tangential vector faces inward; for objects, it faces outward.
    @param border Nx2 np.ndarray representing a border (optional).
    @param objects List of Nx2 np.ndarrays representing objects (optional).
    @return Tuple (point, angle) where:
            - point is a 2-element np.ndarray with the coordinates of the chosen edge point.
            - angle is a float, the clockwise angle from the positive y-axis of the tangential direction.
    """

    ## logic to decide to use objects or border
    use_border_flag = True
    if border is None and objs is None:
        return None
    elif border is not None and objs is not None:
        use_border_flag = generate_random_int(0,1)
    elif border is None:
        use_border_flag = False

    ## border use
    if use_border_flag:
        # get a random point
        point, idx = get_random_coordinate(border)
        #print("point",point)
        # calculate the angle
        angle = tangent_angle_at_point(border,idx)

        return point, angle
        
    ## decide on an object
    object = get_random_array_from_list(objs)
    # get a random point
    point, idx = get_random_coordinate(object)
    #print("point",point)

    # calculate the angle
    angle = tangent_angle_at_point(object,idx)
    angle = angle-np.pi

    return point, angle

# Get a random element in a np.ndarray or list
def get_random_element(array):
    return array[generate_random_int(0,len(array)-1)]