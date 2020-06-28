#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
import unittest
from unittest.mock import Mock
from geometry import Polygon
from perception import Perception
from tests.test_utils import make_square_vertices


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
        self.config.field_elements = [Polygon(make_square_vertices())]
        self.perception = Perception(self.config)

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


if __name__ == '__main__':
    unittest.main()
