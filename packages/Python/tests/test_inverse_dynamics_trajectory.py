import unittest

import numpy as np

import modern_robotics as mr


class TestInverseDynamicsTrajectory(unittest.TestCase):
    def test_fractional_forces_at_integer_positions(self):
        # A unit mass on a vertical prismatic joint under gravity.
        positions = np.array([[0], [1]])
        forces = mr.InverseDynamicsTrajectory(
            positions, np.zeros((2, 1)), [[0.0], [0.5]], [0, 0, -9.81],
            np.zeros((2, 6)), [np.eye(4), np.eye(4)], [np.eye(6)],
            np.array([[0, 0, 0, 0, 0, 1]]).T)
        np.testing.assert_allclose(forces, [[9.81], [10.31]])
