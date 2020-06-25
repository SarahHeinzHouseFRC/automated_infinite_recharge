#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
import unittest
from unittest.mock import Mock
from geometry import Polygon
from perception import Perception
from tests.test_utils import make_square_vertices


class TestPerception(unittest.TestCase):
    def setUp(self):
        self.config = Mock()
        self.config.field_elements = [Polygon(make_square_vertices())]
        self.perception = Perception(self.config)

    def test_robot_centered_on_field_gives_back_same_points(self):
        # Arrange
        self.vehicle_state = {
            'x': 0,
            'y': 0,
            'theta': 0,
            'lidarSweepCartesian': np.array([[1, 0]])
        }

        # Act
        self.perception.vehicle_frame_to_world_frame(self.vehicle_state)

        expected = self.vehicle_state['lidarSweepCartesian']
        actual = self.vehicle_state['lidarSweepWorld']

        # Assert
        self.assertTrue(np.equal(expected, actual).all())


if __name__ == "__main__":
    unittest.main()
