import unittest
from trajsim2d_core.twodmanip import PlanarManipulator
from trajsim2d_core.calculations import calculate_static_torque, calculate_base_wrench_force, evaluate_trajectory, Trajectory
import numpy as np


class test_calculate_static_torque(unittest.TestCase):
    def setUp(self):
        pass
    
    def test_one_link(self):
        
        link_length = 2.0
        link_masses = 1.5
        g = -9.81
        base_offset = 0.5
        
        arm = PlanarManipulator(n=1,base_offset=base_offset, link_lengths=[link_length], link_masses=[0.0, link_masses])
        
        q = 0.0
        tau = calculate_static_torque(arm, arm.base_tf, [q])
        
        print("Torque for one link at  ", np.rad2deg(q), "  radians:", tau[0])
        
        self.assertAlmostEqual(tau[0], 0.0, places=5)
        
        q = np.pi / 2
        tau = calculate_static_torque(arm, arm.base_tf, [q])
        tau_calc = link_length * (link_masses * g)
        
        print("Torque for one link at  ", np.rad2deg(q), "  degrees:", tau[0], ". Expected:", tau_calc)
        
        self.assertAlmostEqual(tau[0], tau_calc, places=5)
        
        
        q = np.pi / 4
        tau = calculate_static_torque(arm, arm.base_tf, [q])
        tau_calc = link_length * np.sin(q) * (link_masses * g)
        
        print("Torque for one link at  ", np.rad2deg(q), "  degrees:", tau[0], ". Expected:", tau_calc)
        
        self.assertAlmostEqual(tau[0], tau_calc, places=5)
        
        theta = np.pi / 4
        base_tf = np.array([[np.cos(theta), -np.sin(theta), 0.0],
                            [np.sin(theta),  np.cos(theta), 0.0],
                            [0.0,  0.0, 1.0]])
        
        q = 0.0
        tau = calculate_static_torque(arm, base_tf, [q])
        
        print("Torque for base at  ", np.rad2deg(q), "  degrees:", tau[0], ". Expected:", -tau_calc)
        
        self.assertAlmostEqual(tau[0], -tau_calc, places=5)
        
        
        
        
                
    def test_two_link(self):
        link_length = [2.0, 1.5]
        link_masses = [1.5, 1.0]
        base_offset = 0.5
        g = -9.81
        
        arm = PlanarManipulator(n=2,base_offset=base_offset, link_lengths=link_length, link_masses=[0.0] + link_masses)
        
        q = [0.0, 0.0]
        tau = calculate_static_torque(arm, arm.base_tf, q)
        
        print("Torque for two links at 0 radians:", tau)
        
        self.assertAlmostEqual(tau[0], 0.0, places=5)
        self.assertAlmostEqual(tau[1], 0.0, places=5)
        
        q = [np.pi / 4, 0.0]
        tau = calculate_static_torque(arm, arm.base_tf, q)
        
        tau1_calc = (link_length[1] + link_length[0]) * np.sin(q[0]) * (link_masses[1] * g) + link_length[0] * np.sin(q[0]) * (link_masses[0] * g)
        tau2_calc = link_length[1] * np.sin(q[0]) * (link_masses[1] * g)
        
        print("Torque for two links at  ", np.rad2deg(q), "  degrees:", tau, ". Expected:", [tau1_calc, tau2_calc])
        
        self.assertAlmostEqual(tau[0], tau1_calc, places=5)
        self.assertAlmostEqual(tau[1], tau2_calc, places=5)
        
        q = [0.0,0.0]
        
        theta = np.pi / 4
        base_tf = np.array([[np.cos(theta), -np.sin(theta), 0.0],
                            [np.sin(theta),  np.cos(theta), 0.0],
                            [0.0,  0.0, 1.0]])
        tau = calculate_static_torque(arm, base_tf, q)
        
        print("Torque for base at  ", np.rad2deg(q), "  degrees, links at 0 radians:", tau, ". Expected:", [-tau1_calc, -tau2_calc])
        
        self.assertAlmostEqual(tau[0], -tau1_calc, places=5)
        self.assertAlmostEqual(tau[1], -tau2_calc, places=5)
        
        q = [np.pi / 4, np.pi/4]
        tau = calculate_static_torque(arm, arm.base_tf, q)
        
        tau1_calc = (link_length[0] * np.sin(q[0]) + link_length[1] * np.sin(q[0] + q[1])) * (link_masses[1] * g) + link_length[0] * np.sin(q[0]) * (link_masses[0] * g)
        tau2_calc = link_length[1] * np.sin(q[0] + q[1]) * (link_masses[1] * g)
        
        print("Torque for two links at ", np.rad2deg(q), " degrees:", tau, ". Expected:", [tau1_calc, tau2_calc])
        
        self.assertAlmostEqual(tau[0], tau1_calc, places=5)
        self.assertAlmostEqual(tau[1], tau2_calc, places=5)
        
    def test_bad_inputs(self):
        link_length = [2.0, 1.5]
        link_masses = [0.0,1.5, 1.0]
        g = -9.81
        
        arm = PlanarManipulator(n=2,base_offset=0.0, link_lengths=link_length, link_masses=link_masses)
        
        q = [0.0]
        
        with self.assertRaises(ValueError):
            tau = calculate_static_torque(arm, arm.base_tf, q)
        
        q = [0.0, 0.0, 0.0]
        
        with self.assertRaises(ValueError):
            tau = calculate_static_torque(arm, arm.base_tf, q)
        
        bad_base_tf = np.eye(4)
        
        with self.assertRaises(ValueError):
            tau = calculate_static_torque(arm, bad_base_tf, [0.0, 0.0])
            
            
            
class test_calculate_base_wrench_force(unittest.TestCase):
    def setUp(self):
        pass

    def test_one_link_static(self):
        link_length = 2.0
        link_masses = [0.4,1.5]
        base_offset = 0.5
        g = -9.81

        arm = PlanarManipulator(
            n=1,
            base_offset=base_offset,
            link_lengths=[link_length],
            link_masses=link_masses
        )

        q = [0.0]

        W = calculate_base_wrench_force(
            manip=arm,
            base_tf=arm.base_tf,
            q=q
        )

        expected_force_y = link_masses[0] * g + link_masses[1] * g
        
        print("Base wrench for one link at ", q, " radians:", W, ". Expected force Y:", expected_force_y)

        self.assertAlmostEqual(W[0], 0.0, places=5)          # Fx
        self.assertAlmostEqual(W[1], expected_force_y, places=5)  # Fy
        self.assertAlmostEqual(W[2], 0.0, places=5)          # Base torque
        
        beta = np.pi / 4
        base_tf = np.array([
            [np.cos(beta), -np.sin(beta), 0.0],
            [np.sin(beta),  np.cos(beta), 0.0],
            [0.0,           0.0,          1.0]
        ])
        
        W = calculate_base_wrench_force(
            manip=arm,
            base_tf=base_tf,
            q=q
        )
        
        # Gravity force expressed in base frame
        expected_fx = -sum(link_masses) * g * np.sin(-beta)
        expected_fy =  sum(link_masses) * g * np.cos(-beta)

        # tau of joint_1
        tau1_calc = link_length * np.sin(-beta) * (link_masses[1] * g)
        
        # joint_1 tau effect at base
        tau1_calc += (base_offset * np.sin(-beta) * (link_masses[0] * g))
        
        # tau effect of end effector
        tau2_calc = (base_offset + link_length) * np.sin(-beta) * link_masses[1] * g
        
        expected_tau = tau1_calc + tau2_calc
        
        
        print("Base wrench for one link at ", q, " radians with base rotated 45 degrees:", W, ". Expected force:", (expected_fx, expected_fy), " Expected torque:", expected_tau)
        
        self.assertAlmostEqual(W[0], expected_fx, places=5)   # Fx
        self.assertAlmostEqual(W[1], expected_fy, places=5)   # Fy
        self.assertAlmostEqual(W[2], expected_tau, places=5)  # Base torque
        
    def test_two_link_static(self):
        link_length = [2.0, 1.5]
        link_masses = [0.4, 1.5, 0.2]  # last mass is for the end-effector
        base_offset = 0.5
        g = -9.81

        arm = PlanarManipulator(
            n=2,
            base_offset=base_offset,
            link_lengths=link_length,
            link_masses=link_masses
        )

        q = [0.0, 0.0]

        W = calculate_base_wrench_force(
            manip=arm,
            base_tf=arm.base_tf,
            q=q
        )

        # Total expected force in Y is sum of all link masses times gravity
        expected_force_y = sum(link_masses) * g

        print("Base wrench for two links at ", q, " radians:", W, ". Expected force Y:", expected_force_y)

        self.assertAlmostEqual(W[0], 0.0, places=5)           # Fx
        self.assertAlmostEqual(W[1], expected_force_y, places=5)  # Fy
        self.assertAlmostEqual(W[2], 0.0, places=5)           # Base torque
        
        
        q = [0.1, -0.1]
        beta = np.pi / 4
        
        base_tf = np.array([
            [np.cos(beta), -np.sin(beta), 0.0],
            [np.sin(beta),  np.cos(beta), 0.0],
            [0.0,           0.0,          1.0]
        ])
        
        W = calculate_base_wrench_force(
            manip=arm,
            base_tf=base_tf,
            q=q
        )
        
        # Gravity force expressed in base frame
        expected_fx = -sum(link_masses) * g * np.sin(-beta)
        expected_fy =  sum(link_masses) * g * np.cos(-beta)
        
        # calculate joint torque for joint 1
        tau1_calc = (
            (
                link_length[0] * np.sin(-beta + q[0]) +
                link_length[1] * np.sin(-beta + q[0] + q[1])
            ) * (link_masses[2] * g)
            +
            link_length[0] * np.sin(-beta + q[0]) * (link_masses[1] * g)
        )

        # joint 1 tau effect at base
        tau1_calc += (
            base_offset * np.sin(-beta) * (link_masses[0] * g)
        )
              
        # calculate joint torque for joint 2
        tau2_calc = (
            link_length[1]
            * np.sin(-beta + q[0] + q[1])
            * (link_masses[2] * g)
        )

        # joint 2 tau effect at base
        tau2_calc += (
            (
                base_offset * np.sin(-beta) +
                link_length[0] * np.sin(-beta + q[0])
            ) * (link_masses[1] * g)
        )
        
        
        # tau effect of end effector
        tau3_calc = (
            (
                base_offset * np.sin(-beta) +
                link_length[0] * np.sin(-beta + q[0]) +
                link_length[1] * np.sin(-beta + q[0] + q[1])
            ) * (link_masses[2] * g)
        )
        
        expected_tau = tau1_calc + tau2_calc + tau3_calc
        
        print("Base wrench for two links at ", q, " radians with base rotated 45 degrees:", W, ". Expected force:", (expected_fx, expected_fy), " Expected torque:", expected_tau)
        
        self.assertAlmostEqual(W[0], expected_fx, places=5)   # Fx
        self.assertAlmostEqual(W[1], expected_fy, places=5)   # Fy
        self.assertAlmostEqual(W[2], expected_tau, places=5)  # Base torque
        
        
class TestTrajectoryEvaluation(unittest.TestCase):
    def setUp(self):
        # Simple 1-link manipulator
        self.link_length = [2.0]
        self.link_mass = [0.4, 1.5]  # last is EE
        self.base_offset = 0.5
        self.g = -9.81

        # Create a simple PlanarManipulator
        self.manip = PlanarManipulator(
            n=1,
            base_offset=self.base_offset,
            link_lengths=self.link_length,
            link_masses=self.link_mass
        )

        # Simple trajectory: 3 time points
        self.time = np.array([0.0, 0.1, 0.2])
        self.q = np.zeros((3, 1))  # joint at 0 radians
        self.base_tf = np.eye(3)
        self.attachment_end = None

        # Trajectory object
        self.traj = Trajectory(
            time=self.time,
            q=self.q,
            base_tf=self.base_tf,
            attachment_end=self.attachment_end
        )

    def test_evaluate_trajectory_static(self):
        # Evaluate trajectory
        evaluate_trajectory(self.traj, self.manip)

        # Check shapes
        N, n = self.q.shape
        self.assertEqual(self.traj.qdot.shape, (N-1, n))
        self.assertEqual(self.traj.qdotdot.shape, (N-2, n))
        self.assertEqual(self.traj.tau.shape, (N, n))
        self.assertEqual(self.traj.base_wrench.shape, (N, 3))
        self.assertEqual(self.traj.in_collision.shape, (N,))

        # Check static torque / base wrench at q=0
        expected_force_y = sum(self.link_mass) * self.g
        
        for i in range(N):
            # Base wrench Y component
            self.assertAlmostEqual(self.traj.base_wrench[i][1], expected_force_y, places=5)
            
            # Not in collision
            self.assertFalse(self.traj.in_collision[i])
        
    
if __name__ == '__main__':
    unittest.main()