import numpy as np
import unittest
import geometry as geom
from geometry import Node, Grid


def sign(value):
    if value > 0:
        return +1
    elif value < 0:
        return -1
    return 0


class TestNodeClass(unittest.TestCase):
    def setUp(self):
        self.position1 = (-1, 25)
        self.position2 = (14, 4)
        self.node1 = Node(self.position1)
        self.node2 = Node(self.position2)

    def test_init_node_creates_node_with_specified_position(self):
        # Assert.
        self.assertEqual(self.node1.position, self.position1)

    def test_clear_node_clears_occupied_flag_and_removes_parent(self):
        # Arrange.
        self.node1.parent = self.node2
        self.node1.occupied = True

        # Act.
        self.node1.clear()

        # Assert.
        self.assertEqual(self.node1.parent, None)
        self.assertFalse(self.node1.occupied)


class TestGridClass(unittest.TestCase):
    def setUp(self):
        self.height = 10
        self.width = 10
        self.cell_resolution = 1
        self.origin = (0, 0)

        self.grid = Grid(self.width, self.height, self.cell_resolution, self.origin)

    def test_init_grid_creates_grid_with_specified_params(self):
        # Assert
        self.assertEqual(self.width, self.grid.width)
        self.assertEqual(self.height, self.grid.height)
        self.assertEqual(self.cell_resolution, self.grid.cell_resolution)
        self.assertEqual(self.origin, self.grid.origin)

    def test_init_grid_creates_grid_with_correct_number_of_cols_and_rows(self):
        # Arrange.
        expected_num_rows = int(self.height / self.cell_resolution)
        expected_num_cols = int(self.width / self.cell_resolution)

        # Assert
        self.assertEqual(expected_num_rows, self.grid.num_rows)
        self.assertEqual(expected_num_cols, self.grid.num_cols)

    def test_init_grid_creates_n_rows_by_n_cols_array(self):
        # Arrange.
        expected_num_rows = int(self.height / self.cell_resolution)
        expected_num_cols = int(self.width / self.cell_resolution)

        # Act.
        self.assertEqual(expected_num_rows, self.grid.num_rows)
        self.assertEqual(expected_num_cols, self.grid.num_cols)


class TestCounterclockwise(unittest.TestCase):
    def setUp(self):
        self.p1 = np.array((0, 0))
        self.p2 = np.array((0, 1))
        self.p3 = np.array((0, 2))
        self.p4 = np.array((1, 0))

        self.p1_to_p2 = self.p2 - self.p1
        self.p1_to_p3 = self.p3 - self.p1
        self.p1_to_p4 = self.p4 - self.p1

    def test_ccw_returns_zero_when_points_are_collinear(self):
        # Act.
        expected_ccw_value = sign(np.cross(self.p1_to_p2, self.p1_to_p3))
        actual_ccw_value = sign(geom.ccw(self.p1, self.p2, self.p3))

        # Assert.
        self.assertEqual(expected_ccw_value, actual_ccw_value)

    def test_ccw_returns_positive_value_when_angle_formed_has_positive_orientation(
            self,
    ):
        # Act.
        expected_sign = sign(np.cross(self.p1_to_p2, self.p1_to_p4))
        actual_sign = sign(geom.ccw(self.p2, self.p1, self.p4))

        # Assert.
        self.assertEqual(expected_sign, actual_sign)

    def test_ccw_returns_negative_value_when_angle_formed_has_negative_orientation(
            self,
    ):
        # Act.
        expected_sign = sign(np.cross(self.p1_to_p4, self.p1_to_p2))
        actual_sign = sign(geom.ccw(self.p4, self.p1, self.p2))

        # Assert.
        self.assertEqual(expected_sign, actual_sign)


if __name__ == '__main__':
    unittest.main()
