import unittest
from trajsim2d_core.twodmanip import PlanarManipulator
from trajsim2d_core.calculations import calculate_static_torque
import numpy as np


class test_calculate_static_torque(unittest.TestCase):
    def setUp(self):
        pass
    
    def test_one_link(self):
        
        link_length = 2.0
        link_mass = 1.5
        g = -9.81
        
        arm = PlanarManipulator(n=1,base_offset=0.0, link_lengths=[link_length], link_masses=[link_mass])
        
        q = 0.0
        tau = calculate_static_torque(arm, arm.base_tf, [q])
        
        print("Torque for one link at 0 radians:", tau[0])
        
        self.assertAlmostEqual(tau[0], 0.0, places=5)
        
        q = np.pi / 2
        tau = calculate_static_torque(arm, arm.base_tf, [q])
        tau_calc = link_length * (link_mass * g)
        
        print("Torque for one link at 90 degrees:", tau[0], ". Expected:", tau_calc)
        
        self.assertAlmostEqual(tau[0], tau_calc, places=5)
        
        
        q = np.pi / 4
        tau = calculate_static_torque(arm, arm.base_tf, [q])
        tau_calc = link_length * np.sin(q) * (link_mass * g)
        
        print("Torque for one link at 45 degrees:", tau[0], ". Expected:", tau_calc)
        
        self.assertAlmostEqual(tau[0], tau_calc, places=5)
        
        theta = np.pi / 4
        base_tf = np.array([[np.cos(theta), -np.sin(theta), 0.0],
                            [np.sin(theta),  np.cos(theta), 0.0],
                            [0.0,  0.0, 1.0]])
        
        q = 0.0
        tau = calculate_static_torque(arm, base_tf, [q])
        
        print("Torque for base at 45 degrees:", tau[0], ". Expected:", -tau_calc)
        
        self.assertAlmostEqual(tau[0], -tau_calc, places=5)
        
                
    def test_two_link(self):
        link_length = [2.0, 1.5]
        link_mass = [1.5, 1.0]
        g = -9.81
        
        arm = PlanarManipulator(n=2,base_offset=0.0, link_lengths=link_length, link_masses=link_mass)
        
        q = [0.0, 0.0]
        tau = calculate_static_torque(arm, arm.base_tf, q)
        
        print("Torque for two links at 0 radians:", tau)
        
        self.assertAlmostEqual(tau[0], 0.0, places=5)
        self.assertAlmostEqual(tau[1], 0.0, places=5)
        
        q = [np.pi / 4, 0.0]
        tau = calculate_static_torque(arm, arm.base_tf, q)
        
        tau1_calc = (link_length[1] + link_length[0]) * np.sin(q[0]) * (link_mass[1] * g) + link_length[0] * np.sin(q[0]) * (link_mass[0] * g)
        tau2_calc = link_length[1] * np.sin(q[0]) * (link_mass[1] * g)
        
        print("Torque for two links at [45,0] degrees:", tau, ". Expected:", [tau1_calc, tau2_calc])
        
        self.assertAlmostEqual(tau[0], tau1_calc, places=5)
        self.assertAlmostEqual(tau[1], tau2_calc, places=5)
        
        q = [0.0,0.0]
        
        theta = np.pi / 4
        base_tf = np.array([[np.cos(theta), -np.sin(theta), 0.0],
                            [np.sin(theta),  np.cos(theta), 0.0],
                            [0.0,  0.0, 1.0]])
        tau = calculate_static_torque(arm, base_tf, q)
        
        print("Torque for base at 45 degrees, links at 0 radians:", tau, ". Expected:", [-tau1_calc, -tau2_calc])
        
        self.assertAlmostEqual(tau[0], -tau1_calc, places=5)
        self.assertAlmostEqual(tau[1], -tau2_calc, places=5)
        
        q = [np.pi / 4, np.pi/4]
        tau = calculate_static_torque(arm, arm.base_tf, q)
        
        tau1_calc = (link_length[0] * np.sin(q[0]) + link_length[1] * np.sin(q[0] + q[1])) * (link_mass[1] * g) + link_length[0] * np.sin(q[0]) * (link_mass[0] * g)
        tau2_calc = link_length[1] * np.sin(q[0] + q[1]) * (link_mass[1] * g)
        
        print("Torque for two links at [45,45] degrees:", tau, ". Expected:", [tau1_calc, tau2_calc])
        
        self.assertAlmostEqual(tau[0], tau1_calc, places=5)
        self.assertAlmostEqual(tau[1], tau2_calc, places=5)
        
    def test_bad_inputs(self):
        link_length = [2.0, 1.5]
        link_mass = [1.5, 1.0]
        g = -9.81
        
        arm = PlanarManipulator(n=2,base_offset=0.0, link_lengths=link_length, link_masses=link_mass)
        
        q = [0.0]
        
        with self.assertRaises(ValueError):
            tau = calculate_static_torque(arm, arm.base_tf, q)
        
        q = [0.0, 0.0, 0.0]
        
        with self.assertRaises(ValueError):
            tau = calculate_static_torque(arm, arm.base_tf, q)
        
        bad_base_tf = np.eye(4)
        
        with self.assertRaises(ValueError):
            tau = calculate_static_torque(arm, bad_base_tf, [0.0, 0.0])