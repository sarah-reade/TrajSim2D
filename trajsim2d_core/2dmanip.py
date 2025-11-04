###############################################################################
## @file 2dmanip.py
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
## >>> from trajsim2d_core.2dmanip import PlanarManipulator
## >>> arm = PlanarManipulator(num_joints=3, link_lengths=[1.0, 1.0, 0.5])
## >>> q = [0.0, 1.0, 0.5]
## >>> pos = arm.forward_kinematics(q)
###############################################################################

# Imports
import numpy as np
from trajsim2d_core.utils import generate_random_number, generate_random_int    

class PlanarManipulator:
    """
    @brief Represents a 2D planar robotic manipulator composed of multiple links and joints.
    @details
    This class encapsulates the parameters of a simple planar manipulator, including
    the widths, lengths, and joint sizes for each link. It can generate random configurations
    when none are provided by the user.
    """

    def __init__(self, base_offset= None, link_width=None, link_lengths=None, joint_radius=None):
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
            self.base_offset,self.link_width,self.link_lengths,self.joint_radius = self.generate_random_arm(base_offset,link_width,link_lengths,joint_radius)

        ## Calculate Joint Limits
        self.joint_limit = self.calculate_joint_limits(link_width,joint_radius)


    def generate_random_arm(self, base_offset=None,link_width=None, link_lengths=None, joint_radius=None):
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
        if base_offset is None:
            base_offset = generate_random_number(0.0,4.0)
        if link_width is None:
            link_width = generate_random_number(0.01,0.2)
        if link_lengths is None:
            link_lengths= generate_random_number(0.1,4,generate_random_int(2,10))
        if joint_radius is None:
            joint_radius= generate_random_number(link_width,link_width*2)
        return link_width, link_lengths, joint_radius
    

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

    def generate_random_config(self, border= None, objects = None, attempts=100):
        config = generate_random_number(-self.joint_limit,self.joint_limit,len(self.link_lengths))
        counter = 1
        while counter < attempts and self.in_collision(config,border,objects):
            config = generate_random_number(-self.joint_limit,self.joint_limit,len(self.link_lengths))
        
        return config

    def in_collision(self,config,border= None,objects = None):
        ## Check for self collision
        if self.in_self_collision(config):
            return True
        
        ## Check for border collision

        ## Check for object collision

        return False

    def in_self_collision(self,config):
        return 