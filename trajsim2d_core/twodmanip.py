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
import xml.etree.ElementTree as ET
from trajsim2d_core.utils import generate_random_number, generate_random_int , getRectAnchor, getRectRotPoint, getRectAngle   
from trajsim2d_core.collision import detect_any_collisions_bounded, detect_any_collisions
from matplotlib.patches import Polygon, Rectangle, Circle
from dataclasses import dataclass


@dataclass
class JointLimits:
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0

@dataclass
class EndEffector:
    """Physical properties of the manipulator's end effector."""

    adhesion: float = 0.0
    width: float = 0.0
    friction: float = 0.0

class PlanarManipulator:
    """
    @brief Represents a 2D planar robotic manipulator composed of multiple links and joints.
    @details
    This class encapsulates the parameters of a simple planar manipulator, including
    the widths, lengths, and joint sizes for each link. It can generate random configurations
    when none are provided by the user.
    """
    

    def __init__(
        self,
        base_tf=None,
        base_offset=None,
        link_width=None,
        link_lengths=None,
        joint_radius=None,
        link_masses=None,
        n=None,
        max_velocity=None,
        adhesion=None,
        ee_width=None,
        friction=None,
        filename=None,
    ):
        """
        @brief Initialize a planar manipulator.
        @param filename Path to a TrajSim2D URDF file to load. When provided,
                        the remaining constructor inputs are ignored.
        @param link_width (float or np.ndarray) Width or widths of the manipulator links.
        @param link_lengths (np.ndarray) Lengths of each link.
        @param joint_radius (float) Radius of the joints between links.
        @param max_velocity (float) Maximum joint velocity. If omitted, a random
                                  value between 0.01 and 0.5 is used.
        @param adhesion (float) End-effector adhesion force. If omitted, a
                               random value between 1.1 and 1.5 times the
                               robot mass is used.
        @param width (float) End-effector width. If omitted, a random value
                            between 2 and 2.5 times the link width is used.
        @param friction (float) End-effector friction coefficient. If omitted,
                               a random value between 0.6 and 0.8 is used.
        @details
        If no dimensions are provided, the manipulator is initialized with random geometry
        using `generate_random_arm()`.
        """
        if filename is not None:
            loaded_manipulator = type(self).load_from_urdf(filename)
            self.__dict__.update(loaded_manipulator.__dict__)
            return

        self.CLIP_ENDS_DEFAULT = 0.01
        
        if base_offset is None or link_width is None or link_lengths is None or joint_radius is None or n is None or link_masses is None:
            self.base_offset,self.link_width,self.link_lengths,self.joint_radius, self.link_masses, self.n = self.generate_random_arm(base_offset,link_width,link_lengths,joint_radius, link_masses, n)
        else:
            self.base_offset = base_offset
            self.link_width = link_width
            self.link_lengths = link_lengths
            self.joint_radius = joint_radius
            self.link_masses = link_masses
            self.n = n

        ## Calculate Joint Limits
        self.joint_limits = self.calculate_joint_limits(max_velocity)
        
        ## Calculate end-effector properties.
        self.end_effector = self.generate_end_effector(
            adhesion, ee_width, friction
        )

        ## Generate base tf if none
        self.base_tf = base_tf
        if base_tf is None:
            self.base_tf = np.eye(3)

    def save_to_urdf(self, filename):
        """
        @brief Save the manipulator and all of its configurable attributes to URDF.
        @details
        Standard URDF links and joints describe the arm structure. A custom
        `trajsim2d` metadata block stores simulator-specific values that URDF
        does not define, including dimensions, masses, limits, transforms, and
        end-effector properties.

        @param filename Path of the URDF file to create.
        """
        robot = ET.Element("robot", {"name": "trajsim2d_manipulator"})
        metadata = ET.SubElement(robot, "trajsim2d")

        def add_value(parent, name, value):
            element = ET.SubElement(parent, name)
            element.text = str(float(value))
            return element

        def add_array(parent, name, value):
            element = ET.SubElement(parent, name)
            element.text = " ".join(
                str(float(item)) for item in np.asarray(value).ravel()
            )
            return element

        add_value(metadata, "base_offset", self.base_offset)
        add_array(metadata, "link_width", self.link_width)
        add_value(metadata, "joint_radius", self.joint_radius)
        add_value(metadata, "max_velocity", self.joint_limits.velocity)
        add_value(metadata, "joint_position_limit", self.joint_limits.position)
        add_value(metadata, "joint_torque_limit", self.joint_limits.torque)
        add_value(metadata, "clip_ends_default", self.CLIP_ENDS_DEFAULT)

        lengths = ET.SubElement(metadata, "link_lengths")
        lengths.text = " ".join(str(float(value)) for value in self.link_lengths)
        masses = ET.SubElement(metadata, "link_masses")
        masses.text = " ".join(str(float(value)) for value in self.link_masses)

        base_transform = ET.SubElement(metadata, "base_transform")
        base_transform.text = " ".join(
            str(float(value)) for value in np.asarray(self.base_tf).ravel()
        )

        end_effector = ET.SubElement(metadata, "end_effector")
        add_value(end_effector, "adhesion", self.end_effector.adhesion)
        add_value(end_effector, "ee_width", self.end_effector.width)
        add_value(end_effector, "friction", self.end_effector.friction)

        # URDF boxes are aligned with the simulator's local +Y link axis.
        # They are clipped around joints so the cylindrical joint geometry is
        # not overlapped by the link geometry.
        thin_depth = 0.01

        def add_box_geometry(link, length, centre_y, width):
            length = max(float(length), 1e-6)
            visual = ET.SubElement(link, "visual")
            ET.SubElement(visual, "origin", {"xyz": f"0 {centre_y} 0", "rpy": "0 0 0"})
            geometry = ET.SubElement(visual, "geometry")
            ET.SubElement(
                geometry,
                "box",
                {"size": f"{float(width)} {length} {thin_depth}"},
            )
            collision = ET.SubElement(link, "collision")
            ET.SubElement(collision, "origin", {"xyz": f"0 {centre_y} 0", "rpy": "0 0 0"})
            collision_geometry = ET.SubElement(collision, "geometry")
            ET.SubElement(
                collision_geometry,
                "box",
                {"size": f"{float(width)} {length} {thin_depth}"},
            )

        def add_joint_geometry(link):
            visual = ET.SubElement(link, "visual")
            geometry = ET.SubElement(visual, "geometry")
            ET.SubElement(
                geometry,
                "cylinder",
                {"radius": str(float(self.joint_radius)), "length": str(thin_depth)},
            )
            collision = ET.SubElement(link, "collision")
            collision_geometry = ET.SubElement(collision, "geometry")
            ET.SubElement(
                collision_geometry,
                "cylinder",
                {"radius": str(float(self.joint_radius)), "length": str(thin_depth)},
            )

        base_link = ET.SubElement(robot, "link", {"name": "base_link"})
        base_length = max(float(self.base_offset) - float(self.joint_radius), 1e-6)
        base_width = float(np.asarray(self.link_width).flat[0])
        add_box_geometry(base_link, base_length, base_length / 2.0, base_width)

        for index in range(self.n):
            parent_name = "base_link" if index == 0 else f"link_{index}"
            child_name = f"link_{index + 1}"
            child_link = ET.SubElement(robot, "link", {"name": child_name})
            width_values = np.asarray(self.link_width).ravel()
            width = float(width_values[0] if width_values.size == 1 else width_values[index])
            if index < self.n - 1:
                link_length = float(self.link_lengths[index]) - 2.0 * float(self.joint_radius)
                add_box_geometry(child_link, link_length, float(self.link_lengths[index]) / 2.0, width)
            else:
                link_length = float(self.link_lengths[index]) - float(self.joint_radius)
                add_box_geometry(child_link, link_length, (float(self.link_lengths[index])+float(self.joint_radius))/2.0, width)
                
                
                                
            add_joint_geometry(child_link)

            joint = ET.SubElement(
                robot,
                "joint",
                {"name": f"joint_{index + 1}", "type": "revolute"},
            )
            ET.SubElement(joint, "parent", {"link": parent_name})
            ET.SubElement(joint, "child", {"link": child_name})
            joint_origin = float(self.base_offset) if index == 0 else float(self.link_lengths[index - 1])
            ET.SubElement(
                joint,
                "origin",
                {"xyz": f"0 {joint_origin} 0", "rpy": "0 0 0"},
            )
            ET.SubElement(joint, "axis", {"xyz": "0 0 1"})
            ET.SubElement(
                joint,
                "limit",
                {
                    "lower": str(-self.joint_limits.position),
                    "upper": str(self.joint_limits.position),
                    "effort": str(self.joint_limits.torque),
                    "velocity": str(self.joint_limits.velocity),
                },
            )

        tree = ET.ElementTree(robot)
        ET.indent(tree, space="  ")
        tree.write(filename, encoding="utf-8", xml_declaration=False)

    @classmethod
    def load_from_urdf(cls, filename):
        """
        @brief Load a manipulator and its attributes from a TrajSim2D URDF.
        @details
        This function reads the custom `trajsim2d` metadata block written by
        `save_to_urdf()`. Standard URDF files without this metadata are rejected
        because they do not contain enough information to reconstruct the
        simulator's manipulator model.

        @param filename Path of the URDF file to load.
        @return PlanarManipulator reconstructed from the saved attributes.
        @raises ValueError If the URDF does not contain TrajSim2D metadata.
        """
        root = ET.parse(filename).getroot()
        metadata = root.find("trajsim2d")
        if metadata is None:
            raise ValueError("URDF does not contain TrajSim2D manipulator metadata")

        def read_value(parent, name):
            element = parent.find(name)
            if element is None or element.text is None:
                raise ValueError(f"URDF metadata is missing '{name}'")
            return float(element.text)

        def read_values(parent, name):
            element = parent.find(name)
            if element is None or element.text is None:
                raise ValueError(f"URDF metadata is missing '{name}'")
            return np.fromstring(element.text, sep=" ")

        link_lengths = read_values(metadata, "link_lengths")
        link_masses = read_values(metadata, "link_masses")
        link_width_values = read_values(metadata, "link_width")
        link_width = (
            float(link_width_values[0])
            if link_width_values.size == 1
            else link_width_values
        )
        transform_values = read_values(metadata, "base_transform")
        if transform_values.size != 9:
            raise ValueError("URDF base_transform must contain 9 values")

        end_effector = metadata.find("end_effector")
        if end_effector is None:
            raise ValueError("URDF metadata is missing 'end_effector'")

        manipulator = cls(
            base_tf=transform_values.reshape(3, 3),
            base_offset=read_value(metadata, "base_offset"),
            link_width=link_width,
            link_lengths=link_lengths,
            joint_radius=read_value(metadata, "joint_radius"),
            link_masses=link_masses,
            n=len(link_lengths),
            max_velocity=read_value(metadata, "max_velocity"),
            adhesion=read_value(end_effector, "adhesion"),
            ee_width=read_value(end_effector, "ee_width"),
            friction=read_value(end_effector, "friction"),
        )
        manipulator.CLIP_ENDS_DEFAULT = read_value(
            metadata, "clip_ends_default"
        )
        manipulator.joint_limits.position = read_value(
            metadata, "joint_position_limit"
        )
        manipulator.joint_limits.torque = read_value(
            metadata, "joint_torque_limit"
        )
        return manipulator

    def generate_end_effector(self, adhesion=None, width=None, friction=None):
        """
        @brief Generate the end-effector properties for the manipulator.
        @details
        This function creates an `EndEffector` object. Any omitted property is
        generated from the robot's physical properties or from the configured
        default range:
        - adhesion is between 1.1 and 1.5 times the total robot mass;
        - width is between 2 and 2.5 times the link width;
        - friction is between 0.6 and 0.8.

        Explicit property values are preserved when provided.

        @param adhesion End-effector adhesion force, or `None` to generate it.
        @param width End-effector width, or `None` to generate it.
        @param friction End-effector friction coefficient, or `None` to
                        generate it.

        @return EndEffector containing adhesion, width, and friction values.
        """
        robot_mass = np.sum(np.asarray(self.link_masses))
        if adhesion is None:
            adhesion = generate_random_number(1.1 * robot_mass, 1.5 * robot_mass)
        if width is None:
            width = generate_random_number(
                2.0 * self.link_width, 2.5 * self.link_width
            )
        if friction is None:
            friction = generate_random_number(0.6, 0.8)

        return EndEffector(
            adhesion=adhesion,
            width=width,
            friction=friction,
        )

    def generate_random_arm(self, base_offset=None,link_width=None, link_lengths=None, joint_radius=None, link_masses=None, n=None):
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
            # Collect lengths of all provided (not None) arrays
            
            lengths = [len(arr) for arr in [link_lengths, link_masses[:-1] if link_masses is not None else None] 
                       if arr is not None]

            if not lengths:
                # No info given → pick random
                n = generate_random_int(2, 10)
            else:
                # Check consistency among provided arrays
                if len(set(lengths)) > 1:
                    raise ValueError(
                        f"Inconsistent lengths among provided manipulator inputs: {lengths}"
                    )
                n = lengths[0]
                
        if base_offset is None:
            base_offset = generate_random_number(self.CLIP_ENDS_DEFAULT*4,0.4)
        if link_width is None:
            link_width = generate_random_number(self.CLIP_ENDS_DEFAULT*2,0.2)
        if link_lengths is None:
            link_lengths= generate_random_number(link_width,1,n)
        if joint_radius is None:
            joint_radius= generate_random_number(link_width/2,np.min(link_lengths)/2)
        if link_masses is None:
            link_masses = generate_random_number(0.1, 5.0, n+1)
            

        # Print all generated values after initialization
        # print(
        #     f"base_offset: {base_offset}, "
        #     f"link_width: {link_width}, "
        #     f"link_lengths: {link_lengths}, "
        #     f"joint_radius: {joint_radius}"
        # )

        return base_offset, link_width, link_lengths, joint_radius, link_masses, n
    
    def calculate_joint_limits(self,max_velocity=None):
        """
        @brief Calculate the position, velocity, and torque limits for the manipulator.
        @details
        This function creates a `JointLimits` object and sets the position limit
        using the link width and joint radius. The maximum joint velocity is
        taken from the supplied value, or randomly generated between 0.01 and
        0.5 when no value is supplied. The same maximum torque is assigned to
        every joint.

        @param max_velocity Maximum joint velocity, or `None` to generate one.

        @return JointLimits containing the calculated position, velocity, and
                torque limits.
        """
        joint_limits = JointLimits()
        joint_limits.position = self.calculate_joint_position_limit(self.link_width,self.joint_radius)
        joint_limits.velocity = self.calculate_joint_velocity_limit(max_velocity)
        joint_limits.torque = self.calculate_joint_torque_limit()
        
        return joint_limits

    def calculate_joint_velocity_limit(self,max_velocity=None):
        """
        @brief Calculate the maximum velocity limit shared by all joints.

        @param max_velocity Maximum joint velocity, or `None` to generate a
                            random value between 0.01 and 0.5.

        @return Maximum joint velocity.
        """
        if max_velocity is None:
            return generate_random_number(0.01, 0.5)
        return max_velocity

    def calculate_joint_torque_limit(self):
        """
        @brief Calculate the maximum torque limit shared by all joints.
        @details
        The first joint is assumed to support the complete arm horizontally.
        Its gravitational torque is calculated from each link mass and the
        horizontal distance to that link's centre of mass. The shared torque
        limit is set to 1.5 times this first-joint torque.

        @return Maximum joint torque.
        """
        link_lengths = np.asarray(self.link_lengths)
        link_masses = np.asarray(self.link_masses)[1:]
        centre_of_mass_distances = np.cumsum(link_lengths) - link_lengths / 2
        first_joint_torque = np.sum(
            link_masses * 9.81 * centre_of_mass_distances
        )
        return 1.5 * first_joint_torque

    def calculate_joint_position_limit(self,link_width,joint_radius):
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
        half_beta = np.arctan2(link_width/2,joint_radius)
        beta = 2*half_beta
        alpha = np.pi - beta
        return alpha

    def generate_random_config(self, border= None, objs = [], base_transform=None, attempts=100, convex_boundary=None):
        config = generate_random_number(-self.joint_limits.position,self.joint_limits.position,len(self.link_lengths))
        counter = 1
        while counter < attempts and self.in_collision(config,border,objs,base_transform=base_transform,convex_boundary=convex_boundary):
            config = generate_random_number(-self.joint_limits.position,self.joint_limits.position,len(self.link_lengths))
            counter += 1

        return self.in_collision(config,border,objs,base_transform=base_transform,convex_boundary=convex_boundary),config

    def in_collision(self,config,border= None,objs = [],base_transform=None,convex_boundary=None):
        
        
        ## make geometry
        arm_geometry=self.make_arm_geometry(config,base_tf=base_transform,clip_ends=self.CLIP_ENDS_DEFAULT)
        
        [self.collision_state, self.collision_list] = detect_any_collisions_bounded(border,arm_geometry,objs,convex_boundary=convex_boundary)
        
        #print("Number of collisions detected:", len(self.collision_list))
        
        ## Check for collisions
        return self.collision_state

    def make_arm_geometry(self,config,base_tf=None,clip_ends=0.01):
        if base_tf is None:
            base_tf=self.base_tf

        tfs = self.forward_kinematics(base_tf,config)
        link_tfs = self.link_forward_kinematics(tfs)

        self.joint_circles = self.make_joint_circles(tfs)
        self.link_rectangles = self.make_link_rectangles(link_tfs,clip_ends=clip_ends)

        return self.joint_circles + self.link_rectangles
    
    def forward_kinematics(self,base_tf,config):
        """
        Compute forward kinematics for a planar linkage.
        base_tf: 3x3 base transform (homogeneous)
        config: array of joint angles [θ₁, θ₂, ...]
        Returns list of transforms (one per joint)
        """

        tfs = []
        config = [-x for x in config]  # Invert angles for correct direction

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
                [c_i,  -s_i, 0],
                [s_i,  c_i, 0],
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

    def make_link_rectangles(self,link_tfs,clip_ends=0.1):
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
            xy=getRectAnchor(tf,self.link_width,self.base_offset - self.joint_radius - 2*clip_ends),
            width=self.link_width,
            height=self.base_offset - self.joint_radius - 2*clip_ends,
            angle=getRectAngle(tf)
        )]

        # link rectangles
        
        rectangles += [
            Rectangle(
                xy=getRectAnchor(tf,self.link_width,self.link_lengths[i] - 2 * self.joint_radius - 2*clip_ends),
                width=self.link_width,
                height=self.link_lengths[i] - 2 * self.joint_radius- 2*clip_ends,
                angle=getRectAngle(tf)
            )
            for i, tf in enumerate(link_tfs[1:-1])  # enumerate for i
        ]  
        
        
        # EE link 
        rectangles += [Rectangle(
            xy=getRectAnchor(link_tfs[-1],self.link_width,self.link_lengths[-1] - 2*self.joint_radius - 2*clip_ends),
            width=self.link_width,
            height=np.maximum(0.0,self.link_lengths[-1] - self.joint_radius - 2*clip_ends),
            angle=getRectAngle(link_tfs[-1])
        )]

        return rectangles

    def print_parameters(self):
        """
        @brief Print the manipulator's parameters to the console.
        """
        print("Planar Manipulator Parameters:")
        print(f"  Number of Links: {self.n}")
        print(f"  Link Widths: {self.link_width}")
        print(f"  Link Lengths: {self.link_lengths}")
        print(f"  Joint Radius: {self.joint_radius}")
        print(f"  Joint Limit (radians): ±{self.joint_limits.position}")
        print(f"  Joint Maximum Velocity: {self.joint_limits.velocity}")
        print(f"  Joint Maximum Torque: {self.joint_limits.torque}")
        print(f"  End-Effector Adhesion: {self.end_effector.adhesion}")
        print(f"  End-Effector Width: {self.end_effector.width}")
        print(f"  End-Effector Friction: {self.end_effector.friction}")