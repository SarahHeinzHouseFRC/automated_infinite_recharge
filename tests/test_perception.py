#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
import unittest
from unittest.mock import Mock
from geometry import Polygon
from perception import Perception
from tests.test_utils import *

BALL_RADIUS = 0.0889


class TestSegmentation(unittest.TestCase):
    def setUp(self):
        self.config = Mock()
        self.config.field_elements = [Polygon(make_square_vertices(side_length=1, center=(0,0)))]
        self.config.outer_wall = Polygon(make_square_vertices(side_length=2, center=(0,0)))
        self.perception = Perception(self.config)

    def test_subtract_background(self):
        vehicle_state = {
            'lidarSweep': [[0.75, 0.75], [1, 1]]
        }

        self.perception.subtract_background(vehicle_state)

        actual = vehicle_state['lidarSweepMask']
        expected = np.array([True, False], dtype=bool)
        np.testing.assert_array_equal(expected, actual)

    def test_clustering_with_3n_distant_points_produces_n_clusters(self):
        vehicle_state = {
            'lidarSweep': np.array([[0, 0], [0, 0], [0, 0], [1, 0], [1, 0], [1, 0], [2, 0], [2, 0], [2, 0]]),
            'lidarSweepMask': np.ones(shape=(9,), dtype=bool)
        }
        self.perception.cluster(vehicle_state)

        expected = 3
        actual = len(vehicle_state['clusters'])
        self.assertEqual(expected, actual)

    def test_clustering_with_close_points_produces_one_cluster(self):
        vehicle_state = {
            'lidarSweep': np.array([[0, 0], [0, 0], [0, 0], [0.1, 0], [0.1, 0], [0.1, 0], [0.2, 0], [0.2, 0], [0.2, 0]]),
            'lidarSweepMask': np.ones(shape=(9,), dtype=bool)
        }
        self.perception.cluster(vehicle_state)

        expected = 1
        actual = len(vehicle_state['clusters'])
        self.assertEqual(expected, actual)


class TestClassification(unittest.TestCase):
    def setUp(self):
        self.config = Mock()
        self.config.field_elements = [Polygon(make_square_vertices(side_length=2, center=(0,0)))]
        self.config.ball_radius = BALL_RADIUS
        self.perception = Perception(self.config)

    def test_classification_of_circular_points_right_size(self):
        vehicle_state = {
            'clusters': [np.array(make_circular_vertices(radius=BALL_RADIUS, center=(0,0), num_pts=6))]
        }

        self.perception.classify(vehicle_state)

        self.assertEqual(len(vehicle_state['balls']), 1)
        self.assertEqual(len(vehicle_state['obstacles']), 0)

    def test_classification_of_circular_points_wrong_size(self):
        vehicle_state = {
            'clusters': [np.array(make_circular_vertices(radius=BALL_RADIUS+0.1, center=(0,0), num_pts=6))]
        }

        self.perception.classify(vehicle_state)

        self.assertEqual(len(vehicle_state['balls']), 0)
        self.assertEqual(len(vehicle_state['obstacles']), 1)

    def test_classification_of_non_circular_points(self):
        vehicle_state = {
            'clusters': [np.array(make_linear_vertices(start=[0, 0], end=[10, 10], num_pts=10))]
        }

        self.perception.classify(vehicle_state)

        self.assertEqual(len(vehicle_state['balls']), 0)
        self.assertEqual(len(vehicle_state['obstacles']), 1)


class TestRun(unittest.TestCase):
    def setUp(self):
        self.config = Mock()
        self.config.field_elements = [Polygon(make_square_vertices(side_length=2, center=(-5, -5)))]
        self.config.ball_radius = BALL_RADIUS
        self.perception = Perception(self.config)

    def test_run(self):
        obstacle = make_linear_vertices(start=(5,5), end=(6,6), num_pts=20)
        ball = make_circular_vertices(radius=BALL_RADIUS, center=(1,1), num_pts=8)
        lidar_sweep = list()
        lidar_sweep.extend(obstacle)
        lidar_sweep.extend(ball)

        x = 0
        y = 0
        theta = 0
        ingested_balls = 0
        vehicle_state = {
            'x': x,
            'y': y,
            'theta': theta,
            'lidarSweep': np.array(lidar_sweep),
            'numIngestedBalls': ingested_balls
        }

        world_state = self.perception.run(vehicle_state)

        actual_balls = world_state['balls']
        actual_first_ball = actual_balls[0]
        actual_others = world_state['obstacles']
        expected_balls = [(1, 1)]
        expected_others = [((5.0, 5.0), (5.95, 5.95))]
        expected_first_ball = expected_balls[0]

        self.assertEqual(len(expected_balls), len(actual_balls))
        self.assertAlmostEqual(expected_first_ball[0], actual_first_ball[0])
        self.assertAlmostEqual(expected_first_ball[1], actual_first_ball[1])

        self.assertEqual(expected_others, actual_others)


if __name__ == '__main__':
    unittest.main()
