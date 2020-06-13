import numpy as np
import unittest

from algorithm import Node
from algorithm import Grid


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
        self.height = 5
        self.width = 7
        self.cell_resolution = 0.01
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

    #def test_init_grid_creates_n_rows_by_n_cols_array(self):
    #    # Arrange.
    #    expected_num_rows = int(self.height / self.cell_resolution)
    #    expected_num_cols = int(self.width / self.cell_resolution)

    #    expected_array = np.empty(
    #        shape=(expected_num_rows, expected_num_cols), dtype=Node
    #    )

    #    # Act.
    #    self.assertTrue(np.array_equal(expected_array, self.grid.grid))


if __name__ == "__main__":
    unittest.main()
