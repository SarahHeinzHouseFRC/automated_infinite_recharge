#
# Copyright (c) 2020 FRC Team 3260
#

import unittest
from unittest.mock import Mock
from planning import Planning
from geometry import Polygon
from tests.test_utils import *


class TestMotionPlanning(unittest.TestCase):
    def setUp(self):
        self.config = Mock()
        self.config.field_elements = []
        self.config.occupancy_grid_width = 6
        self.config.occupancy_grid_height = 6
        self.config.occupancy_grid_cell_resolution = 1
        self.config.occupancy_grid_origin = (0, 0)
        self.config.occupancy_grid_dilation_kernel_size = 3

        self.pose = ((-2.5, -2.5), 0)
        self.goal = (2.5, 2.5)

        self.planning = Planning(self.config)

    def test_motion_planning_with_empty_world(self):
        world_state = {
            'obstacles': {
                'others': [],
            },
            'pose': self.pose,
            'goal': self.goal,
        }

        self.planning.motion_planning(world_state)

        expected_trajectory_length = 6
        actual_trajectory_length = len(world_state['trajectory'])
        expected_occupancy_grid = np.zeros(shape=(6,6))
        actual_occupancy_grid = world_state['grid'].occupancy

        self.assertEqual(expected_trajectory_length, actual_trajectory_length)
        np.testing.assert_array_equal(expected_occupancy_grid, actual_occupancy_grid)

    def test_motion_planning_avoids_static_obstacle(self):
        self.planning.static_obstacles = [Polygon(make_square_vertices(side_length=1.5, center=(0,0)))]

        world_state = {
            'obstacles': {
                'others': [],
            },
            'pose': self.pose,
            'goal': self.goal,
        }

        self.planning.motion_planning(world_state)

        expected_occupancy_grid = np.zeros(shape=(6,6), dtype=np.uint8)
        expected_occupancy_grid[1:5,1:5] = np.ones(shape=(4,4))
        actual_occupancy_grid = world_state['grid'].occupancy
        expected_trajectory_length = 10

        actual_trajectory_length = len(world_state['trajectory'])
        np.testing.assert_array_equal(expected_occupancy_grid, actual_occupancy_grid)
        self.assertEqual(expected_trajectory_length, actual_trajectory_length)

    def test_motion_planning_avoids_dynamic_obstacle(self):
        world_state = {
            'obstacles': {
                'others': [((-0.5, -0.5), (0.5, 0.5))],
            },
            'pose': self.pose,
            'goal': self.goal,
        }

        self.planning.motion_planning(world_state)

        expected_occupancy_grid = np.zeros(shape=(6, 6), dtype=np.uint8)
        expected_occupancy_grid[1:5, 1:5] = np.ones(shape=(4, 4))
        actual_occupancy_grid = world_state['grid'].occupancy
        expected_trajectory_length = 10
        actual_trajectory_length = len(world_state['trajectory'])

        np.testing.assert_array_equal(expected_occupancy_grid, actual_occupancy_grid)
        self.assertEqual(expected_trajectory_length, actual_trajectory_length)

    def test_motion_planning_returns_none_when_no_feasible_trajectory(self):
        world_state = {
            'obstacles': {
                'others': [((1, 1), (2, 2))],
            },
            'pose': self.pose,
            'goal': self.goal,
        }

        goal_cell = self.planning.occupancy_grid.get_cell(self.goal)
        self.planning.occupancy_grid.occupancy[goal_cell.indices] = 1
        self.planning.motion_planning(world_state)

        self.assertIsNone(world_state['trajectory'])

    def test_motion_planning_returns_trivial_plan_when_goal_reached(self):
        world_state = {
            'obstacles': {
                'others': [],
            },
            'pose': self.pose,
            'goal': self.pose[0],
        }

        self.planning.motion_planning(world_state)

        expected = 2
        actual = len(world_state['trajectory'])
        self.assertEqual(expected, actual)


if __name__ == '__main__':
    unittest.main()
