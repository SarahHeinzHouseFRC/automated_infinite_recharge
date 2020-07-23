#
# Copyright (c) 2020 FRC Team 3260
#

import unittest
from unittest.mock import Mock
import numpy as np
from tests.test_utils import *
from geometry import Polygon
from sweep_builder import SweepBuilder


class TestPreprocessSweep(unittest.TestCase):
    def setUp(self):
        self.sweep_builder = SweepBuilder()

    def test_filter_empty_rays(self):
        vehicle_state = {
            'lidarPoints': [[0, 0, 0], [0, 0, 1], [0, 0, 2]]
        }
        self.sweep_builder.filter_empty_rays(vehicle_state)

        expected = np.array([[0, 0, 1], [0, 0, 2]])
        actual = vehicle_state['lidarPointsFiltered']

        np.testing.assert_array_equal(expected, actual)

    def test_spherical_to_cartesian(self):
        vehicle_state = {
            'lidarPointsFiltered': np.array([[0, 0, 1], [np.pi/2, 0, 1]])
        }
        self.sweep_builder.spherical_to_cartesian(vehicle_state)

        expected = np.array([[1, 0], [0, 1]])
        actual = vehicle_state['lidarPointsCartesian']

        np.testing.assert_array_almost_equal(expected, actual)


class TestLocalization(unittest.TestCase):
    def setUp(self):
        self.sweep_builder = SweepBuilder()

    def test_vehicle_centered_on_field_gives_back_same_points(self):
        # Arrange
        vehicle_state = {
            'x': 0,
            'y': 0,
            'theta': 0,
            'lidarPointsCartesian': np.array([[1, 0]])
        }

        # Act
        self.sweep_builder.vehicle_frame_to_world_frame(vehicle_state)

        expected = vehicle_state['lidarPointsCartesian']
        actual = vehicle_state['lidarPointsWorld']

        # Assert
        self.assertTrue(np.equal(expected, actual).all())

    def test_vehicle_facing_left_returns_points_rotated_by_90_deg(self):
        # Arrange
        vehicle_state = {
            'x': 0,
            'y': 0,
            'theta': np.pi/2,
            'lidarPointsCartesian': np.array([[1, 0]])
        }

        # Act
        self.sweep_builder.vehicle_frame_to_world_frame(vehicle_state)

        expected = np.array([[0, 1]])
        actual = vehicle_state['lidarPointsWorld']

        # Assert
        np.testing.assert_almost_equal(expected, actual)


class TestSweepBuilder(unittest.TestCase):
    def setUp(self):
        self.sweep_builder = SweepBuilder()

    def test_full_sweep_empty_when_azimuth_hasnt_crossed_zero(self):
        vehicle_state = {
            'lidarPointsFiltered': np.array([[0, 0, 15], [np.pi/2, 0, 15], [np.pi, 0, 15]]),
            'lidarPointsWorld': np.array([[15, 0], [0, 15], [-15, 0]])
        }
        self.sweep_builder.build_sweep(vehicle_state)

        expected_partial_sweep = [[15, 0], [0, 15], [-15, 0]]
        actual_partial_sweep = self.sweep_builder.partial_sweep
        expected_full_sweep = []
        actual_full_sweep = self.sweep_builder.full_sweep

        np.testing.assert_array_equal(expected_partial_sweep, actual_partial_sweep)
        np.testing.assert_array_equal(expected_full_sweep, actual_full_sweep)

    def test_full_sweep_when_azimuth_crosses_zero_again(self):
        vehicle_state = {
            'lidarPointsFiltered': np.array([[np.pi, 0, 15], [3*np.pi/2, 0, 15], [0, 0, 15]]),
            'lidarPointsWorld': np.array([[-15, 0], [0, -15], [15, 0]])
        }
        self.sweep_builder.build_sweep(vehicle_state)

        expected_partial_sweep = [[15, 0]]
        actual_partial_sweep = self.sweep_builder.partial_sweep
        expected_full_sweep = np.array([[-15, 0], [0, -15]])
        actual_full_sweep = self.sweep_builder.full_sweep

        np.testing.assert_array_equal(expected_partial_sweep, actual_partial_sweep)
        np.testing.assert_array_equal(expected_full_sweep, actual_full_sweep)


class TestRun(unittest.TestCase):
    def setUp(self):
        self.sweep_builder = SweepBuilder()

    def test_partial_sweep(self):
        vehicle_state = {
            'lidarPoints': [[0, 0, 15], [np.pi/2, 0, 15], [np.pi, 0, 15]],
            'x': 0,
            'y': 0,
            'theta': 0,
        }

        ret = self.sweep_builder.run([vehicle_state])

        expected = None
        actual = ret

        self.assertEqual(expected, actual)

    def test_full_sweep(self):
        vehicle_state = {
            'lidarPoints': [[0, 0, 15], [np.pi/2, 0, 15], [np.pi, 0, 15], [3*np.pi/2, 0, 15], [0, 0, 15]],
            'x': 0,
            'y': 0,
            'theta': 0,
        }

        ret = self.sweep_builder.run([vehicle_state])

        expected = np.array([[15, 0], [0, 15], [-15, 0], [0, -15]])
        actual = ret['lidarSweep']

        np.testing.assert_array_almost_equal(expected, actual)


if __name__ == '__main__':
    unittest.main()
