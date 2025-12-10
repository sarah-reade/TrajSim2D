###############################################################################
## @file twodmanip.py
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
## >>> from trajsim2d_core.twodmanip import PlanarManipulator
## >>> arm = PlanarManipulator(num_joints=3, link_lengths=[1.0, 1.0, 0.5])
## >>> q = [0.0, 1.0, 0.5]
## >>> pos = arm.forward_kinematics(q)
###############################################################################

# Imports
import numpy as np
from trajsim2d_core.utils import generate_random_number, generate_random_int , getRectAnchor, getRectRotPoint, getRectAngle   
from trajsim2d_core.collision import detect_any_collisions_bounded, detect_any_collisions
from matplotlib.patches import Polygon, Rectangle, Circle

class PlanarManipulator:
    """
    @brief Represents a 2D planar robotic manipulator composed of multiple links and joints.
    @details
    This class encapsulates the parameters of a simple planar manipulator, including
    the widths, lengths, and joint sizes for each link. It can generate random configurations
    when none are provided by the user.
    """

    def __init__(self, base_tf=None, base_offset= None, link_width=None, link_lengths=None, joint_radius=None, n=None):
        """
        @brief Initialize a planar manipulator.
        @param link_width (float or np.ndarray) Width or widths of the manipulator links.
        @param link_lengths (np.ndarray) Lengths of each link.
        @param joint_radius (float) Radius of the joints between links.
        @details
        If no dimensions are provided, the manipulator is initialized with random geometry
        using `generate_random_arm()`.
        """
        
        if link_width is None or link_lengths is None or joint_radius:
            self.base_offset,self.link_width,self.link_lengths,self.joint_radius, self.n = self.generate_random_arm(base_offset,link_width,link_lengths,joint_radius, n)

        ## Calculate Joint Limits
        self.joint_limit = self.calculate_joint_limits(self.link_width,self.joint_radius)

        ## Generate base tf if none
        self.base_tf = base_tf
        if base_tf is None:
            self.base_tf = np.eye(3)


    def generate_random_arm(self, base_offset=None,link_width=None, link_lengths=None, joint_radius=None, n=None):
        """
        @brief Generate random link width and length arrays for a multi-link arm.
        @details
        This function creates a random width and lengths for each link of a simple
        robotic arm. The number of links is chosen randomly between 2 and 10.
        Each link width and length is generated using uniform random sampling.

        @return Tuple (base_offset,link_width, link_length, joint_radius):
            - base_offset: float of random length in range [0.0, 4.0]
            - link_width: float of random width in range [0.01, 1.0]
            - link_length: np.ndarray of random lengths in range [0.1, 4.0]
            - joint_radius: float of random joint radius in range [0.01, 1.0]
        """
        if n is None:
            n = generate_random_int(3,6)
        if base_offset is None:
            base_offset = generate_random_number(0.0,4.0)
        if link_width is None:
            link_width = generate_random_number(0.01,0.2)
        if link_lengths is None:
            link_lengths= generate_random_number(0.1,1,n)
        if joint_radius is None:
            joint_radius= generate_random_number(link_width/2,link_width)
            

        # Print all generated values after initialization
        # print(
        #     f"base_offset: {base_offset}, "
        #     f"link_width: {link_width}, "
        #     f"link_lengths: {link_lengths}, "
        #     f"joint_radius: {joint_radius}"
        # )

        return base_offset, link_width, link_lengths, joint_radius, n
    

    def calculate_joint_limits(self,link_width,joint_radius):
        """
        @brief Calculates the angular limits for a joint based on the link width and joint radius.
        
        This function computes the maximum angular deviation of a joint such that 
        two tangential lines of length link_width/2 meet at a point. The angle is 
        determined using basic trigonometry with the arctangent function.
        
        @param link_width The total width of the link connected to the joint.
        @param joint_radius The radius of the circular joint.
        
        @return Angle from straight
        
        @note The computed angle assumes the link attaches tangentially to the 
            circular joint.
        
        """
        half_alpha = np.arctan2(link_width/2,joint_radius)
        alpha = 2*half_alpha
        return alpha

    def generate_random_config(self, border= None, objs = [], attempts=100):
        config = generate_random_number(-self.joint_limit,self.joint_limit,len(self.link_lengths))
        counter = 1
        while counter < attempts and self.in_collision(config,border,objs):
            config = generate_random_number(-self.joint_limit,self.joint_limit,len(self.link_lengths))
            counter += 1
        
        return config

    def in_collision(self,config,border= None,objs = []):
        
        ## make geometry
        arm_geometry=self.make_arm_geometry(config)
        
        ## Check for collisions
        collision_flag, collision_list = detect_any_collisions(arm_geometry,objs,0.01) ## TODO: CHANGE ME
        if collision_flag: 
            return True

        return False

    def make_arm_geometry(self,config,base_tf=None):
        if base_tf is None:
            base_tf=self.base_tf

        tfs = self.forward_kinematics(base_tf,config)
        link_tfs = self.link_forward_kinematics(tfs)

        self.joint_circles = self.make_joint_circles(tfs)
        self.link_rectangles = self.make_link_rectangles(link_tfs)

        return self.joint_circles + self.link_rectangles
    
    def forward_kinematics(self,base_tf,config):
        """
        Compute forward kinematics for a planar linkage.
        base_tf: 3x3 base transform (homogeneous)
        config: array of joint angles [θ₁, θ₂, ...]
        Returns list of transforms (one per joint)
        """

        tfs = []

        # Base offset (e.g., vertical offset from ground)
        temp_tf = np.eye(3)
        temp_tf[1,2] = self.base_offset
        joint_1_tf  = np.dot(base_tf,temp_tf)
        tfs.append(joint_1_tf)

        # Loop over each joint (the last is the E Position)
        for i in range(len(config)):
            temp_tf = np.eye(3)
            c_i = np.cos(config[i])
            s_i = np.sin(config[i])

            # Local transform for this joint
            rot_tf = np.array([
                [c_i,  s_i, 0],
                [-s_i,  c_i, 0],
                [0,    0,   1]
            ])
            trans_tf = np.array([
                [1, 0, 0],
                [0, 1, self.link_lengths[i]],
                [0, 0, 1]
            ])

            joint_tf = np.dot(tfs[i], np.dot(rot_tf, trans_tf))
            tfs.append(joint_tf)
        
        return tfs
    
    def link_forward_kinematics(self,tfs):
        """
        Compute transforms for link centers based on joint transforms.
        tfs: list of joint transforms
        Returns list of transforms for each link center
        """
        link_tfs = []

        # Add base link tf
        temp_tf = np.eye(3)
        temp_tf[1,2] = -(self.base_offset+self.joint_radius)/2
        base_link_tf = np.dot(tfs[0],temp_tf)
        link_tfs.append(base_link_tf)

        # Add all other links
        for i in range(len(tfs)-1):
            temp_tf[1,2] = -self.link_lengths[i]/2
            link_tf = np.dot(tfs[i+1],temp_tf)
            link_tfs.append(link_tf)

        return link_tfs
    
    def make_joint_circles(self,tfs):
        """
        @brief Generate Circle objects representing the joints of the robotic arm.
        
        Each joint is represented as a Circle centered at the corresponding
        transform in `tfs`, except the last transform (usually the end-effector,
        which is not a joint).

        @param tfs List of 3x3 homogeneous transforms for each joint.
        @return List of Circle objects representing each joint.
        """

        return [Circle([tf[0,2],tf[1,2]],self.joint_radius) for tf in tfs[:-1]]

    def make_link_rectangles(self,link_tfs):
        """
        @brief Generate Rectangle objects representing the links of the robotic arm.

        The base link is treated specially: its length is reduced by one joint radius
        at one end. All other links are reduced by two joint radii to account forGJK for 2D
        the joints at both ends.

        @param link_tfs List of 3x3 homogeneous transforms corresponding to the
                        center of each link.
        @return List of Rectangle objects representing each link.
        """
        tf = link_tfs[0]
        # base rectangle
        rectangles = [Rectangle(
            xy=getRectAnchor(tf,self.link_width,self.base_offset - self.joint_radius),
            width=self.link_width,
            height=self.base_offset - self.joint_radius,
            angle=getRectAngle(tf)
        )]

        # link rectangles
        rectangles += [
            Rectangle(
                xy=getRectAnchor(tf,self.link_width,self.link_lengths[i] - 2 * self.joint_radius),
                width=self.link_width,
                height=self.link_lengths[i] - 2 * self.joint_radius,
                angle=getRectAngle(tf)
            )
            for i, tf in enumerate(link_tfs[1:])  # enumerate for i
        ]  

        return rectangles
