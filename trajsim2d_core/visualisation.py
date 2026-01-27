###############################################################################
## @file visualisation.py
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
import threading
import time
from trajsim2d_core.geometry_canvas import GeometryCanvas
from trajsim2d_core.utils import make_transform_2d, generate_random_number
from trajsim2d_core.environment import generate_random_edge_point
from trajsim2d_core.twodmanip import PlanarManipulator
from trajsim2d_core.collision import detect_any_collisions, detect_any_collisions_bounded, create_convex_boundary_objects
from trajsim2d_core.shape_utils import shape_in_tuple_list
import numpy as np

# Initialise visualisation 
def initialise_visualisation(border=None,objs=None,base_transform=None,arm=None,joint_config_1=None,joint_config_2=None,attempt_max = 10):
    """
    @brief Initialise a 2D visualisation with a border.
    @param border Nx2 np.ndarray defining the boundary.
    @param objs Placeholder for objects (ignored for now).
    @param base_transform Placeholder (ignored for now).
    @param arm 2dmanip Object.
    @param joint_config_1 Placeholder (ignored for now).
    @param joint_config_2 Placeholder (ignored for now).
    @return Figure and axis handles.
    """
    canvas = GeometryCanvas()
    ## Add border
    border_id = 0
    if border is not None:
        border_id = visualise_object(canvas,border,'olive')

    ## Add objects
    object_ids = []
    if objs is not None:
        object_ids = visualise_object(canvas,objs,'black')
    
    ## Add Arm (if one of border and objects exist)
    arm_ids = []
    if arm is not None:
        arm_ids, base_transform, joint_config_1, joint_config_2 = initialise_visualise_arm(canvas,arm,border,objs,base_transform,joint_config_1,joint_config_2,attempt_max=attempt_max)
        base_id = canvas.add_tf(base_transform)
    
    canvas.refresh()

    return canvas, base_transform, border_id, object_ids, arm_ids, joint_config_1, joint_config_2

# Initialise Arm
def initialise_visualise_arm(canvas,arm=PlanarManipulator(),border=None,objs=None,base_transform=None,joint_config_1=None,joint_config_2=None,attempt_max = 10):
    """
    @ brief adds an arm to the canvas
    """
    
    if border is None:
        return [], np.eye(3), None, None
    
    ## generate the base_transform
    if base_transform is None:
        base_transform = make_transform_2d() 

        if border is not None or objs is not None:
            collision = True
            attempts = 0
            convex_border = create_convex_boundary_objects(border) 
            while collision:
                if attempts == attempt_max:
                    attempt_max = 1
                    break 
                
                print(f"Attempt {attempts+1} to place arm without collision.")
                point, angle = generate_random_edge_point(border,objs)
                base_transform = make_transform_2d(point[0],point[1],angle)
                config = [0] * arm.n
                collision = arm.in_collision(config, border,objs,base_transform=base_transform,convex_boundary=convex_border)
                if collision:
                    print("Number of Collisions:", len(arm.collision_list))
                    arm_geometry = arm.make_arm_geometry(config=config,base_tf=base_transform,clip_ends=arm.CLIP_ENDS_DEFAULT)
                    
                    # print("base link", arm_geometry[arm.n])
                    # print("joint 1", arm_geometry[0])
                    
                    if shape_in_tuple_list([arm_geometry[arm.n],arm_geometry[0]],arm.collision_list):
                        
                        attempts += 1
                        continue
                    
                break
    
    ids_1, joint_config_1 = visualise_arm(canvas,arm,base_transform,border,objs,attempt_max,convex_boundary=convex_border,joint_config=joint_config_1,color='blue', alpha=0.5)
    ids_2, joint_config_2 = visualise_arm(canvas,arm,base_transform,border,objs,attempt_max,convex_boundary=convex_border,joint_config=joint_config_2,color='green', alpha=0.5)

    return ids_1 + ids_2, base_transform, joint_config_1, joint_config_2

def visualise_arm(canvas,arm=PlanarManipulator(),base_transform=None,border=None,objs=None,attempt_max=10,convex_boundary=None,joint_config=None,color='red', alpha=0.5):
    
    self_collision = False
    
    if joint_config is None:
        [self_collision, joint_config] = arm.generate_random_config(border,objs,base_transform=base_transform,attempts=attempt_max,convex_boundary=convex_boundary)
        
    arm_geometry = arm.make_arm_geometry(joint_config,base_transform,clip_ends=0.0)
    id = visualise_object(canvas,arm_geometry,color, alpha)
    if self_collision:
        print("Warning: Could not generate a self-collision-free joint configuration.")
        for [shape_1,shape_2,distance] in arm.collision_list:
            id += [visualise_object(canvas,shape_1,'red',alpha=0.5)]
            id += [visualise_object(canvas,shape_2,'red',alpha=0.5)]
            
    return id, joint_config

def visualise_object(canvas,obj,color='red', alpha=0.5):
    return canvas.add_shape(obj,color,alpha)

# update trajectory visualisation
def update_trajectory_visualisation(time,canvas : GeometryCanvas,arm,trajectory,arm_ids,border=None,objs=None):
    """
    @brief Update the visualisation to a specific time in the trajectory.
    @param time Time point to visualise.
    @param canvas GeometryCanvas object.
    @param arm PlanarManipulator object.
    @param trajectory Trajectory object.
    @param arm_ids List of shape IDs for the arm segments.
    @param border Nx2 np.ndarray defining the boundary.
    @param objs List of objects in the environment.
    
    @return new_arm_ids Updated list of shape IDs for the arm segments.
    @return done Boolean indicating if the trajectory has completed.
    """
    
    # Find the index corresponding to the given time
    index = np.searchsorted(trajectory.time, time)
    if index >= len(trajectory.time):
        index = len(trajectory.time) - 1
        return arm_ids, True
    
    # Remove existing arm shapes
    for id in arm_ids:
        if id is not None:
            canvas.remove_shape(id)
        else:
            print("Warning: Tried to remove a None shape id.")
    
    # Visualise arm at the specified time
    config = trajectory.q[index,:]
    
    [new_arm_ids,_] = visualise_arm(canvas,arm,trajectory.base_tf,joint_config=config,color='black', alpha=0.5,border=border,objs=objs)
    
    canvas.refresh()
    
    return new_arm_ids, False


# Sync visualise trajectory
def visualise_trajectory_sync(canvas : GeometryCanvas,arm,trajectory,border=None,objs=None):
    """
    @brief Visualise a trajectory synchronously.
    @param canvas GeometryCanvas object.
    @param arm PlanarManipulator object.
    @param trajectory Trajectory object.
    @param border Nx2 np.ndarray defining the boundary.
    @param objs List of objects in the environment.
    """
    
    # start timer
    start_time = time.time()
    
    arm_ids = []
    while True:
        
        # Update visualisation
        [arm_ids, done] = update_trajectory_visualisation(time.time() - start_time,canvas,arm,trajectory
                                                        ,arm_ids,border=border,objs=objs)
        if done:
            return arm_ids
        
        time.sleep(0.001)

    return arm_ids

# Visualise outputs 

# Wait for visualisation of trajectory to be over