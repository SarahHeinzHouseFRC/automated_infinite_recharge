#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
import unittest
from unittest.mock import Mock
from geometry import Polygon
from perception import Perception
from perception import IN_TO_M
from tests.test_utils import *


class TestPreprocessSweep(unittest.TestCase):
    def setUp(self):
        self.config = Mock()
        self.config.field_elements = [Polygon(make_square_vertices())]
        self.perception = Perception(self.config)

    def test_robot_centered_on_field_gives_back_same_points(self):
        # Arrange
        vehicle_state = {
            'x': 0,
            'y': 0,
            'theta': 0,
            'lidarSweepCartesian': np.array([[1, 0]])
        }

        # Act
        self.perception.vehicle_frame_to_world_frame(vehicle_state)

        expected = vehicle_state['lidarSweepCartesian']
        actual = vehicle_state['lidarSweepWorld']

        # Assert
        self.assertTrue(np.equal(expected, actual).all())


class TestSegmentation(unittest.TestCase):
    def setUp(self):
        self.config = Mock()
        self.config.field_elements = [Polygon(make_square_vertices(side_length=1))]
        self.config.outer_wall = Polygon(make_square_vertices(side_length=2))
        self.perception = Perception(self.config)

    def test_subtract_background(self):
        vehicle_state = {
            'lidarSweepWorld': [[0.75, 0.75], [1, 1]]
        }

        self.perception.subtract_background(vehicle_state)

        actual = vehicle_state['lidarSweepMask']
        expected = np.array([True, False], dtype=bool)
        np.testing.assert_array_equal(expected, actual)

    def test_clustering_with_n_distant_points_produces_n_clusters(self):
        vehicle_state = {
            'lidarSweepWorld': np.array(make_square_vertices()),
            'lidarSweepMask': np.ones(shape=(4,), dtype=bool)
        }
        self.perception.cluster(vehicle_state)

        expected = 4
        actual = len(vehicle_state['clusters'])
        self.assertEqual(expected, actual)

    def test_clustering_with_close_points_produces_one_cluster(self):
        vehicle_state = {
            'lidarSweepWorld': np.array(make_square_vertices(side_length=0.1)),
            'lidarSweepMask': np.ones(shape=(4,), dtype=bool)
        }
        self.perception.cluster(vehicle_state)

        expected = 1
        actual = len(vehicle_state['clusters'])
        self.assertEqual(expected, actual)

    def test_classification_of_circular_points_right_size(self):
        vehicle_state = {
            'clusters': [np.array(make_circular_vertices(radius=3.55*IN_TO_M))]
        }

        self.perception.classify(vehicle_state)

        self.assertEqual(len(vehicle_state['classes']['balls']), 1)
        self.assertEqual(len(vehicle_state['classes']['others']), 0)

    def test_classification_of_circular_points_wrong_size(self):
        vehicle_state = {
            'clusters': [np.array(make_circular_vertices(radius=3.6))]
        }

        self.perception.classify(vehicle_state)

        self.assertEqual(len(vehicle_state['classes']['balls']), 0)
        self.assertEqual(len(vehicle_state['classes']['others']), 1)

    def test_classification_of_non_circular_points(self):
        vehicle_state = {
            'clusters': [np.array(make_linear_vertices(start=[0, 0], end=[10, 10], num_pts=10))]
        }

        self.perception.classify(vehicle_state)

        self.assertEqual(len(vehicle_state['classes']['balls']), 0)
        self.assertEqual(len(vehicle_state['classes']['others']), 1)


if __name__ == '__main__':
    unittest.main()
