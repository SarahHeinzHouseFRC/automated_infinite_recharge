#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
import unittest
from tests.test_utils import *
import geometry as geom
from geometry import Node, OccupancyGrid, Polygon


class TestNodeClass(unittest.TestCase):
    def setUp(self):
        self.position1 = (-1, 25)
        self.position2 = (14, 4)
        self.node1 = Node(self.position1, (1, 1))
        self.node2 = Node(self.position2, (2, 2))

    def test_init_node_creates_node_with_specified_position(self):
        # Assert.
        self.assertEqual(self.node1.position, self.position1)

    def test_clear_node_removes_parent(self):
        # Arrange.
        self.node1.parent = self.node2

        # Act.
        self.node1.clear()

        # Assert.
        self.assertEqual(self.node1.parent, None)


class TestGridClass(unittest.TestCase):
    def setUp(self):
        self.height = 4
        self.width = 4
        self.cell_resolution = 1
        self.origin = (0, 0)

        self.occupancy_grid = OccupancyGrid(self.width, self.height, self.cell_resolution, self.origin)

    def test_init_grid_creates_grid_with_specified_params(self):
        # Assert
        self.assertEqual(self.width, self.occupancy_grid.width)
        self.assertEqual(self.height, self.occupancy_grid.height)
        self.assertEqual(self.cell_resolution, self.occupancy_grid.cell_resolution)
        self.assertEqual(self.origin, self.occupancy_grid.origin)

    def test_init_grid_creates_grid_with_correct_number_of_cols_and_rows(self):
        # Arrange.
        expected_num_rows = int(self.height / self.cell_resolution)
        expected_num_cols = int(self.width / self.cell_resolution)

        # Assert
        self.assertEqual(expected_num_rows, self.occupancy_grid.num_rows)
        self.assertEqual(expected_num_cols, self.occupancy_grid.num_cols)

    def test_init_grid_creates_n_rows_by_n_cols_array(self):
        # Arrange.
        expected_num_rows = int(self.height / self.cell_resolution)
        expected_num_cols = int(self.width / self.cell_resolution)

        # Assert
        self.assertEqual(expected_num_rows, self.occupancy_grid.num_rows)
        self.assertEqual(expected_num_cols, self.occupancy_grid.num_cols)

    def test_clear_node_parents(self):
        self.occupancy_grid.clear_node_parents()

        for col in self.occupancy_grid.grid:
            for cell in col:
                self.assertEqual(cell.parent, None)

    def test_clear(self):
        self.occupancy_grid.clear()

        for col in self.occupancy_grid.grid:
            for cell in col:
                self.assertEqual(cell.parent, None)
                self.assertEqual(self.occupancy_grid.occupancy[cell.indices], False)

    def test_get_cell_valid(self):
        cell = self.occupancy_grid.get_cell((0.5, 0.5))

        expected = self.occupancy_grid.grid[2, 2]
        actual = cell

        self.assertEqual(expected, actual)

    def test_get_cell_out_of_bounds(self):
        cell = self.occupancy_grid.get_cell((10, 10))

        self.assertIsNone(cell)

    def test_insert_rectangular_obstacle(self):
        rect = ((1, 1), (1.5, 1.5))

        self.occupancy_grid.insert_rectangular_obstacle(rect, 1.0)

        expected = [0] * 15 + [1]
        actual = self.occupancy_grid.occupancy.flatten(order='F')
        np.testing.assert_array_equal(expected, actual)

    def test_insert_convex_polygon_with_square_obstacle(self):
        vertices = make_square_vertices(side_length=0.5, center=(1.25, 1.25))
        square = Polygon(vertices)

        self.occupancy_grid.insert_convex_polygon(square, 1.0)

        expected = [0] * 15 + [1]
        actual = self.occupancy_grid.occupancy.flatten(order='F')
        np.testing.assert_array_equal(expected, actual)

    def test_insert_convex_polygon_with_circular_obstacle(self):
        circle = Polygon(make_circular_vertices(radius=1, center=(0, 0), num_pts=8))

        self.occupancy_grid.insert_convex_polygon(circle, 1.0)

        expected = [0] * 16
        expected[5] = 1
        expected[6] = 1
        expected[9] = 1
        expected[10] = 1
        actual = self.occupancy_grid.occupancy.flatten(order='F')
        np.testing.assert_array_equal(expected, actual)

    def test_dilation_of_empty_grid(self):
        self.occupancy_grid.inflate_obstacles(kernel_size=3)

        expected = [0] * 16
        actual = self.occupancy_grid.occupancy.flatten(order='F')

        np.testing.assert_array_equal(expected, actual)

    def test_dilation_of_single_cell(self):
        self.occupancy_grid.occupancy[1][1] = 1
        self.occupancy_grid.inflate_obstacles(kernel_size=3)

        expected = [1]*3 + [0] + [1]*3 + [0] + [1]*3 + [0]*5
        actual = self.occupancy_grid.occupancy.flatten(order='F')

        np.testing.assert_array_equal(expected, actual)

    def test_dilation_with_odd_kernel_throws(self):
        self.assertRaises(ValueError, self.occupancy_grid.inflate_obstacles, kernel_size=6)


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

    def test_ccw_returns_positive_value_when_angle_formed_has_positive_orientation(self):
        # Act.
        expected_sign = sign(np.cross(self.p1_to_p2, self.p1_to_p4))
        actual_sign = sign(geom.ccw(self.p2, self.p1, self.p4))

        # Assert.
        self.assertEqual(expected_sign, actual_sign)

    def test_ccw_returns_negative_value_when_angle_formed_has_negative_orientation(self):
        # Act.
        expected_sign = sign(np.cross(self.p1_to_p4, self.p1_to_p2))
        actual_sign = sign(geom.ccw(self.p4, self.p1, self.p2))

        # Assert.
        self.assertEqual(expected_sign, actual_sign)


class TestBoundingBox(unittest.TestCase):

    def test_bounding_box_square(self):
        """
        Test the bounding_box() function
        """
        # Arrange
        side_length = 4
        points = np.array([[0,0], [0, side_length], [side_length, 0], [side_length, side_length]])

        # Act
        expected = (0,0), (side_length, side_length)
        actual = geom.bounding_box(points)

        # Assert
        self.assertEqual(expected, actual)

    def test_bounding_box_nonconvex(self):
        # Arrange
        points = make_square_vertices(side_length=2, center=(0, 0)) + [[0,0]]

        # Act
        expected = (-1,-1), (1, 1)
        actual = geom.bounding_box(points)

        # Assert
        self.assertEqual(expected, actual)

    def test_bounding_box_no_area(self):
        # Arrange
        points = np.array([[4,4], [4,4], [4,4], [4,4]])

        # Act
        expected = (4,4),(4,4)
        actual = geom.bounding_box(points)

        # Assert
        self.assertEqual(expected, actual)


class TestPolygon(unittest.TestCase):
    def test_convex_polygon_convexity(self):
        # Arrange
        square = Polygon(make_square_vertices(side_length=2, center=(0,0)))
        expected = True
        actual = square.is_convex()
        self.assertEqual(expected, actual)

    def test_nonconvex_polygon_convexity(self):
        vertices = make_square_vertices(side_length=2, center=(0,0)) + [[0,0]]
        nonconvex = Polygon(vertices)

        expected = False
        actual = nonconvex.convex
        self.assertEqual(expected, actual)

    def test_point_is_in_square_polygon(self):
        square = Polygon(make_square_vertices(side_length=2, center=(0,0)))
        # Assert
        expected = True
        actual = square.point_in_convex_polygon((0,0))

        self.assertEqual(expected, actual)

    def test_point_not_in_square_polygon(self):
        # Arrange
        square = Polygon(make_square_vertices(side_length=2, center=(0,0)))

        # Act
        expected = False
        actual = square.point_in_convex_polygon((3,3))

        # Assert
        self.assertEqual(expected, actual)

    def test_point_on_polygon_corner(self):
        # Arrange
        square = geom.Polygon(make_square_vertices(side_length=2, center=(0,0)))

        # Act
        expected = True
        actual = square.point_in_convex_polygon((1,1))

        # Assert
        self.assertEqual(expected, actual)

    def test_point_in_nonconvex_polygon_throws(self):
        # Arrange
        vertices = make_square_vertices(side_length=2, center=(0, 0)) + [[0,0]]
        nonconvex = geom.Polygon(vertices)

        # Assert
        self.assertRaises(ValueError, nonconvex.point_in_convex_polygon, (0,0))

    def test_scale_polygon_by_one_has_same_vertices(self):
        vertices = make_square_vertices(side_length=2, center=(2,2))
        polygon = geom.Polygon(vertices)
        polygon.scale(factor=1)

        expected = vertices
        actual = polygon.vertices
        np.testing.assert_array_equal(expected, actual)

    def test_scale_polygon_by_ten(self):
        polygon = geom.Polygon(make_square_vertices(side_length=2, center=(2,2)))
        polygon.scale(factor=10)

        expected = make_square_vertices(side_length=20, center=(2,2))
        actual = polygon.vertices

        np.testing.assert_array_equal(expected, actual)


class TestConnectedComponents(unittest.TestCase):
    def test_empty_buckets_result_in_one_empty_cc(self):
        buckets = {
            (0, 0): [],
            (1, 0): [],
            (0, 1): [],
            (1, 1): [],
        }

        result = geom.connected_components(buckets)

        expected = 1
        actual = len(result)

        self.assertEqual(expected, actual)

    def test_all_buckets_filled_result_in_one_large_cc(self):
        buckets = {
            (0, 0): [1],
            (1, 0): [2],
            (0, 1): [3],
            (1, 1): [4],
        }

        result = geom.connected_components(buckets)

        expected = 1
        actual = len(result)

        self.assertEqual(expected, actual)

        expected = [1, 2, 3, 4]
        actual = result[0]

        np.testing.assert_array_equal(sorted(expected), sorted(actual))


class TestAStar(unittest.TestCase):
    def setUp(self):
        self.occupancy_grid = OccupancyGrid(width=4, height=4, cell_resolution=1, origin=(0,0))
        self.start = (-1.5, -1.5)
        self.goal = (1.5, 1.5)

    def test_a_star_fails_when_start_occluded(self):
        start_node = self.occupancy_grid.get_cell(self.start)
        self.occupancy_grid.occupancy[start_node.indices] = 1
        result = geom.a_star(self.occupancy_grid, 1.0, self.start, self.goal)
        self.assertIsNone(result)

    def test_a_star_fails_when_goal_occluded(self):
        goal_node = self.occupancy_grid.get_cell(self.goal)
        self.occupancy_grid.occupancy[goal_node.indices] = 1
        result = geom.a_star(self.occupancy_grid, 1.0, self.start, self.goal)
        self.assertIsNone(result)

    def test_a_star_succeeds_when_all_cells_unoccupied(self):
        result = geom.a_star(self.occupancy_grid, 1.0, self.start, self.goal)
        expected = 4
        actual = len(result)
        self.assertEqual(expected, actual)

    def test_a_star_fails_when_goal_unreachable(self):
        self.occupancy_grid.occupancy[2,3] = 1
        self.occupancy_grid.occupancy[2,2] = 1
        self.occupancy_grid.occupancy[3,2] = 1
        result = geom.a_star(self.occupancy_grid, 1.0, self.start, self.goal)
        self.assertIsNone(result)


class TestSmoother(unittest.TestCase):
    def test_trajectory_with_two_points_remains_unchanged(self):
        trajectory = [(0.1, 0), (1, 1)]
        smoothed_trajectory = geom.smooth_trajectory(trajectory)

        expected = trajectory
        actual = smoothed_trajectory

        np.testing.assert_array_equal(expected, actual)

    def test_smoother_removes_extraneous_points(self):
        trajectory = [(0.1, 0), (1, 1), (2, 2), (3, 2)]
        smoothed_trajectory = geom.smooth_trajectory(trajectory)

        expected = [(0.1, 0), (2, 2), (3, 2)]
        actual = smoothed_trajectory

        np.testing.assert_array_equal(expected, actual)


class TestIntersections(unittest.TestCase):
    def test_no_line_line_intersection(self):
        line1 = (0, 0), (10, 10)
        line2 = (1, 1), (11, 11)

        expected = None
        actual = geom.line_line_intersection(line1, line2)

        self.assertEqual(expected, actual)

    def test_line_line_intersection(self):
        line1 = (-10, 0), (10, 0)
        line2 = (0, -10), (0, 10)

        expected = (0, 0)
        actual = geom.line_line_intersection(line1, line2)

        self.assertEqual(expected, actual)

    def test_no_line_rect_intersection(self):
        line = (-10, 10), (10, 10)
        rect = (-5, -5), (5, 5)

        expected = []
        actual = geom.line_rect_intersection(line, rect)

        self.assertEqual(expected, actual)

    def test_line_rect_intersection(self):
        line = (-10, 0), (10, 0)
        rect = (-5, -5), (5, 5)

        expected = [(-5, 0), (5, 0)]
        actual = geom.line_rect_intersection(line, rect)

        self.assertEqual(expected, actual)


class TestCircleFit(unittest.TestCase):
    def test_make_circle(self):
        vertices = make_circular_vertices(radius=2, center=(2, 2), num_pts=3)

        circle = geom.make_circle(vertices)

        expected = ((2, 2), 2)
        expected_center_x = expected[0][0]
        expected_center_y = expected[0][1]
        expected_radius = expected[1]
        actual = circle
        actual_center_x = actual[0][0]
        actual_center_y = actual[0][1]
        actual_radius = actual[1]

        self.assertAlmostEqual(expected_center_x, actual_center_x)
        self.assertAlmostEqual(expected_center_y, actual_center_y)
        self.assertAlmostEqual(expected_radius, actual_radius)

    def test_ransac_circle_fit_on_circular_vertices_with_correct_radius_succeeds(self):
        points = np.array(make_circular_vertices(radius=2, center=(2, 2), num_pts=8))
        result = geom.ransac_circle_fit(points, desired_radius=2, consensus=0.99, tolerance=0.03, iterations=10)

        expected = ((2, 2), 2)
        expected_center_x = expected[0][0]
        expected_center_y = expected[0][1]
        expected_radius = expected[1]
        actual = result
        actual_center_x = actual[0][0]
        actual_center_y = actual[0][1]
        actual_radius = actual[1]

        self.assertAlmostEqual(expected_center_x, actual_center_x)
        self.assertAlmostEqual(expected_center_y, actual_center_y)
        self.assertAlmostEqual(expected_radius, actual_radius)

    def test_ransac_circle_fit_on_circular_vertices_with_wrong_radius_fails(self):
        points = np.array(make_circular_vertices(radius=3, center=(2, 2), num_pts=8))
        result = geom.ransac_circle_fit(points, desired_radius=2, consensus=0.99, tolerance=0.03, iterations=10)

        self.assertIsNone(result)

    def test_ransac_circle_fit_on_linear_vertices_fails(self):
        points = np.array(make_linear_vertices(start=(2,2), end=(5,5), num_pts=8))
        result = geom.ransac_circle_fit(points, desired_radius=2, consensus=0.99, tolerance=0.03, iterations=10)

        self.assertIsNone(result)

    def test_ransac_circle_fit_on_two_points_returns_none(self):
        points = np.array(make_circular_vertices(radius=2, center=(2, 2), num_pts=2))
        result = geom.ransac_circle_fit(points, desired_radius=2, consensus=0.99, tolerance=0.03, iterations=10)
        self.assertIsNone(result)


if __name__ == '__main__':
    unittest.main()
