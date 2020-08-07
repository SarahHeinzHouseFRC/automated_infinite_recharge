#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
from collections import defaultdict
from math import atan2, sqrt
import cv2 as cv


class Node:
    def __init__(self, position, indices):
        """
        :param position: Node's position as tuple(x, y)
        :param indices: Node's indices as tuple(col, row)
        """
        self.position = position
        self.indices = indices
        self.neighbors = set()
        self.parent = None

    def __lt__(self, other):
        return self.position[0] < other.position[0]

    def __str__(self):
        return f'Node{self.indices}'

    def clear(self):
        self.parent = None


class OccupancyGrid:
    def __init__(self, width, height, cell_resolution, origin):
        """
        Constructs a list of nodes comprising this rectangular area.
        Assumes cell_resolution divides evenly into length and width
        :param width: Width of the grid in meters
        :param height: Height of the grid in meters
        :param cell_resolution: Width and height of a single cell
        :param origin: Position of grid center in meters as tuple(x, y)
        """

        # store the resolution of each cell for marking cells later
        self.width = width
        self.height = height
        self.cell_resolution = cell_resolution
        self.origin = origin
        self.num_cols = int(self.width / self.cell_resolution)
        self.num_rows = int(self.height / self.cell_resolution)
        self.grid = np.ndarray((self.num_cols, self.num_rows), dtype=Node)

        self.occupancy = np.zeros(self.grid.shape, dtype=np.float)

        # 1. Create all the nodes
        for col in range(self.num_cols):
            for row in range(self.num_rows):
                x = (col * self.cell_resolution) + (self.cell_resolution / 2) - self.width / 2
                y = (row * self.cell_resolution) + (self.cell_resolution / 2) - self.height / 2
                self.grid[col][row] = Node((x, y), (col, row))

        # 2. Connect all the neighbors
        min_col = 0
        max_col = self.num_cols-1
        min_row = 0
        max_row = self.num_rows-1
        adjacent_cell_cost = self.cell_resolution
        diagonal_cell_cost = self.cell_resolution * sqrt(2)
        for x in range(self.num_cols):
            for y in range(self.num_rows):
                node = self.grid[x][y]
                if x < max_col:
                    node.neighbors.add((self.grid[x+1][y], adjacent_cell_cost))
                if x > min_col:
                    node.neighbors.add((self.grid[x-1][y], adjacent_cell_cost))
                if y < max_row:
                    node.neighbors.add((self.grid[x][y+1], adjacent_cell_cost))
                if y > min_row:
                    node.neighbors.add((self.grid[x][y-1], adjacent_cell_cost))
                if x < max_col and y < max_row:
                    node.neighbors.add((self.grid[x+1][y+1], diagonal_cell_cost))
                if x > min_col and y < max_row:
                    node.neighbors.add((self.grid[x-1][y+1], diagonal_cell_cost))
                if x < max_col and y > min_row:
                    node.neighbors.add((self.grid[x+1][y-1], diagonal_cell_cost))
                if x > min_col and y > min_row:
                    node.neighbors.add((self.grid[x-1][y-1], diagonal_cell_cost))

    def clear_node_parents(self):
        """
        Resets all parents but does not mark cells as unoccupied
        """
        for col in self.grid:
            for cell in col:
                cell.clear()

    def clear(self):
        """
        Resets all parents and marks all nodes as unoccupied
        """
        self.clear_node_parents()
        self.occupancy.fill(0)

    def get_cell(self, point):
        """
        Queries the grid to return the cell containing the given point, or None if out-of-bounds
        :param point: A point as tuple(x, y)
        :return: The node corresponding to the given point
        """
        # Error-checking
        x = point[0]
        y = point[1]
        min_x = self.origin[0] - (self.width / 2)
        max_x = self.origin[0] + (self.width / 2)
        min_y = self.origin[1] - (self.height / 2)
        max_y = self.origin[1] + (self.height / 2)
        if min_x <= x < max_x and min_y <= y < max_y:
            col = int((x - min_x) / self.cell_resolution)
            row = int((y - min_y) / self.cell_resolution)
            return self.grid[col][row]
        else:
            return None

    def get_subgrid(self, bbox):
        """
        Returns the min/max column and row indices corresponding to the given bounding box
        :param bbox: Bounding box in the form ((min_x, min_y), (max_x, max_y))
        :return: Region as (min_col, max_col, min_row, max_row)
        """
        (min_x, min_y), (max_x, max_y) = bbox
        min_col = int((min_x + self.origin[0]) / self.cell_resolution + self.num_cols / 2)
        max_col = int((max_x + self.origin[0]) / self.cell_resolution + self.num_cols / 2)
        min_row = int((min_y + self.origin[1]) / self.cell_resolution + self.num_rows / 2)
        max_row = int((max_y + self.origin[1]) / self.cell_resolution + self.num_rows / 2)
        return min_col, min_row, max_col, max_row

    def get_occupancy(self, pos):
        cell = self.get_cell(pos)
        return self.occupancy[cell.indices]

    def insert_rectangular_obstacle(self, obstacle, growth_factor):
        """
        Finds the grid cells corresponding to the given obstacle and marks them as occupied.
        :param obstacle: A rectangular obstacle in the form ((min_x, min_y), (max_x, max_y))
        :param growth_factor Increase the probability of an obstacle by this amount
        """
        min_col, min_row, max_col, max_row = self.get_subgrid(obstacle)

        for col in range(min_col, max_col+1):
            for row in range(min_row, max_row+1):
                self.occupancy[col][row] += growth_factor

    def insert_convex_polygon(self, polygon, growth_factor):
        """
        Find the grid cells corresponding to each obstacle and marks them as occupied.
        :param polygon: A convex polygon
        :param growth_factor Increase probability of an obstacle by this amount
        """
        # 1. Find the sub grid to iterate over
        min_col, min_row, max_col, max_row = self.get_subgrid(bounding_box(polygon.vertices))
        subgrid = self.grid[min_col:max_col+1, min_row:max_row+1]

        # 2. Iterate over each column
        for col in subgrid:
            col_x, _ = col[0].position
            # 3. In the current col, find all the contact points where the current col intersects with the polygon
            contact_cells = set()
            for i in range(len(polygon.vertices)):
                p1 = polygon.vertices[i - 1]
                p2 = polygon.vertices[i]
                if p1[0] <= col_x <= p2[0] or p2[0] <= col_x <= p1[0]:
                    # go on to the next edge if this one's slope is undefined
                    num = p2[1]-p1[1]
                    den = p2[0]-p1[0]
                    m = num / (den if den else 1e-5)
                    x = col_x
                    y = m*(x - p1[0]) + p1[1]
                    cell = self.get_cell((x, y))
                    if cell is not None:
                        contact_cells.add(cell)

            # 4. Iterate over the contact points marking each cell in this row as occupied/unoccupied
            occupied_flag = 0

            if len(contact_cells) > 1:
                for cell in col:
                    if cell in contact_cells:
                        self.occupancy[cell.indices] += growth_factor  # make sure contact cells are marked as occupied
                        occupied_flag = not occupied_flag
                    elif occupied_flag:
                        self.occupancy[cell.indices] += growth_factor
            else:
                for cell in contact_cells:
                    self.occupancy[cell.indices] += growth_factor  # make sure contact cells are marked as occupied

    def inflate_obstacles(self, kernel_size):
        """
        Expand all occupied cells in the grid by a buffer
        :param kernel_size: Odd integer to use for sliding window
        """
        if kernel_size % 2 != 1:
            raise ValueError("Kernel size must be an odd integer")

        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        self.occupancy = cv.dilate(self.occupancy, kernel)


def a_star(occupancy_grid, occupancy_threshold, start, goal):
    """
    Returns a trajectory from start to goal as a list of Nodes or None if no path is found. The returned trajectory will
    start with the start node and end with the goal node and is guaranteed to have at least a length of 2.
    :param occupancy_grid: Occupancy grid
    :param occupancy_threshold: Threshold for a cell to be considered occupied (0-1)
    :param start: Starting position
    :param goal: Goal position
    :return: List of trajectory points as list(tuple(x, y), ...)
    """
    start_node = occupancy_grid.get_cell(start)
    goal_node = occupancy_grid.get_cell(goal)

    # Trivial case
    if start_node is goal_node:
        return [start, goal]

    start_node.parent = start_node  # Just needs to be something other than None so we don't try to explore it
    queue = [(0, start_node)]

    # Make sure start_node and/or goal are not obstructed
    if occupancy_grid.get_occupancy(start) >= occupancy_threshold or occupancy_grid.get_occupancy(goal) >= occupancy_threshold:
        return None

    while len(queue) > 0 and goal_node.parent is None:
        # Pop a node from the queue
        _, curr_node = queue.pop(0)

        # Run through its neighbors
        for neighbor, neighbor_cost in curr_node.neighbors:
            # If we found the goal, break out
            if neighbor == goal_node:
                neighbor.parent = curr_node
                break

            # If unvisited and unoccupied, add it to the queue
            if neighbor.parent is None and occupancy_grid.occupancy[neighbor.indices] < occupancy_threshold:
                neighbor.parent = curr_node
                cost = neighbor_cost  # g(x) = How much does it cost to travel go to this neighbor from curr_node?
                heuristic = dist(neighbor.position, goal_node.position)  # h(x) = How close is curr_node to the goal?
                score = heuristic + cost  # f(x) = g(x) + h(x)
                queue.append((score, neighbor))
                queue.sort()

    node_path = []
    curr_node = goal_node

    # If no node_path was found, return none
    if goal_node.parent is None:
        return None

    # Iterate backwards from goal
    while curr_node != start_node:
        node_path.insert(0, curr_node)
        curr_node = curr_node.parent

    # Add starting node to node_path
    node_path.insert(0, start_node)

    # Convert list of nodes to a trajectory
    path = [node.position for node in node_path]
    path[0] = start
    path[-1] = goal

    return path


def smooth_trajectory(trajectory):
    """
    Smooths out the kinks in the given trajectory. Any turn in the path that is more than 45 degrees is a "real" turn,
    and any turn that is -45 < theta < 45 degrees is a "kink" or redundant and can be removed.

    E.g. [(0.1, 0), (1, 1), (2, 2), (3, 2)] -> [(0.1, 0), (2, 2), (3, 2)]

    :param trajectory: List of points as tuples
    :return: Same as previous list but with points removed
    """
    # nothing to do if less than 3 points
    if len(trajectory) < 3:
        return trajectory

    smoothed_trajectory = [trajectory[0]]

    for p1,p2,p3 in zip(trajectory[:], trajectory[1:], trajectory[2:]):
        v1 = np.array(p2) - np.array(p1)
        v2 = np.array(p3) - np.array(p2)
        x1 = v1[0]
        y1 = v1[1]
        x2 = v2[0]
        y2 = v2[1]
        theta = atan2(x1 * y2 - y1 * x2, x1 * x2 + y1 * y2)

        TURN_THRESHOLD = np.pi / 4
        if abs(theta) >= TURN_THRESHOLD:
            smoothed_trajectory.append(p2)

    smoothed_trajectory.append(trajectory[-1])
    return smoothed_trajectory


def bounding_box(points):
    """
    Takes a list of points and returns a tuple (min_x, min_y), (max_x, max_y)
    """
    min_x, min_y = np.min(points, axis=0)
    max_x, max_y = np.max(points, axis=0)
    return (min_x, min_y), (max_x, max_y)


class Cell:
    def __init__(self, bucket_id, points):
        self.x = bucket_id[0]
        self.y = bucket_id[1]
        self.visited = False
        self.cluster_id = None
        self.points = points

    def get_neighbors(self):
        # Returns IDs of neighboring cells
        return [(self.x+1,self.y),
                (self.x+1,self.y+1),
                (self.x,self.y+1),
                (self.x-1,self.y+1),
                (self.x-1,self.y),
                (self.x-1,self.y-1),
                (self.x,self.y-1),
                (self.x+1,self.y-1)]


def connected_components(buckets):
    """
    Connects neighboring buckets into connected components. Takes in a dictionary of buckets in the form:
    buckets = {
        (0,0) : list(... points ...),
        (0,1) : list(... points ...),
        ...
    }

    And connects neighboring cells into connected components. The result is returned in the form:
    clusters = [
        np.array(... points ...),
        np.array(... points ...),
        np.array(... points ...),
    ]
    """

    # Convert the dict of buckets dict into a dict of cells
    grid = dict()
    for bucket_id, points in buckets.items():
        cell = Cell(bucket_id, points)
        grid[bucket_id] = cell

    # Connect neighboring cells into connected components
    clusters = defaultdict(list)
    cluster_id = 0
    for root_cell in grid.values():
        # if we've been to a cell, skip it
        if root_cell.cluster_id is not None:
            continue
        root_cell.cluster_id = cluster_id
        # Start a BFS to connect all neighboring cells
        queue = [root_cell]
        while len(queue) > 0:
            cur_cell = queue.pop(0)
            if cur_cell.visited:
                continue
            cur_cell.visited = True
            cur_cell.cluster_id = cluster_id
            clusters[cur_cell.cluster_id].append(cur_cell)

            queue += [grid[n] for n in cur_cell.get_neighbors() if n in grid]

        cluster_id += 1

    new_clusters = []
    for cluster_id, cells in clusters.items():
        this_cluster = []
        for cell in cells:
            this_cluster.extend(cell.points)
        new_clusters.append(np.array(this_cluster))
    return new_clusters


def ransac_circle_fit(points, desired_radius, consensus, tolerance, iterations):
    """
    Takes N points as an Nx2 array and returns the best-fit circle or None.
    :param points: Nx2 numpy array of points
    :param desired_radius: Desired radius of fit circle
    :param consensus: How much of a consensus is needed to declare success, 0 < consensus <= 1
    :param tolerance: Tolerance to be declared an inlier of the circle
    :param iterations: Max number of iterations to try
    :returns: Circle as tuple(tuple(x, y), radius) or None
    """

    # We require at least 3 points to fit a circle
    if len(points) < 3:
        return None

    np.random.seed(10)
    for _ in range(iterations):
        # 1. Take a random sample of 3 points from this cluster
        random_sample = np.random.choice(len(points), 3, replace=False)

        # 2. Fit a circle to those points
        fit_circle = make_circle(points[random_sample[0:3]])

        # 3. Check all other points in the cluster for consensus
        fit_circle_center = fit_circle[0]
        fit_circle_radius = fit_circle[1]

        ball_radius_tolerance = 0.05
        if not abs(fit_circle_radius - desired_radius) <= ball_radius_tolerance:
            continue

        num_inliers = 0
        for point in points:
            if abs(dist(point, fit_circle_center) - fit_circle_radius) <= tolerance:
                num_inliers += 1

        # 4. If we've reached a consensus, then return the circle we've found
        if num_inliers / len(points) >= consensus:
            return fit_circle
    return None


def make_circle(points):
    """
    Constructs a circle from 3 points.
    Source: https://www.w3resource.com/python-exercises/basic/python-basic-1-exercise-39.php
    :param points: Three 2D points as numpy arrays of shape (2,)
    :return: Circle as tuple(tuple(x, y), radius)
    """
    p1 = points[0]
    p2 = points[1]
    p3 = points[2]

    x1 = p1[0]
    x2 = p2[0]
    x3 = p3[0]
    y1 = p1[1]
    y2 = p2[1]
    y3 = p3[1]

    c = (x1 - x2) ** 2 + (y1 - y2) ** 2
    a = (x2 - x3) ** 2 + (y2 - y3) ** 2
    b = (x3 - x1) ** 2 + (y3 - y1) ** 2
    s = 2 * (a * b + b * c + c * a) - (a * a + b * b + c * c)
    s = -1e-5 if -1e5 < s < 0 else 1e-5 if 0 <= s < 1e-5 else s # Prevent divide by zero
    px = (a * (b + c - a) * x1 + b * (c + a - b) * x2 + c * (a + b - c) * x3) / s
    py = (a * (b + c - a) * y1 + b * (c + a - b) * y2 + c * (a + b - c) * y3) / s
    ar = a ** 0.5
    br = b ** 0.5
    cr = c ** 0.5
    den = ((ar + br + cr) * (-ar + br + cr) * (ar - br + cr) * (ar + br - cr))
    den = -1e-5 if -1e5 < den < 0 else 1e-5 if 0 <= den < 1e-5 else den # Prevent divide by 0
    r = ar * br * cr / den ** 0.5

    return (px, py), r


def dist(p1, p2):
    """
    Takes 2 points as array-like and returns the Euclidean distance between them.
    """
    x = p2[0] - p1[0]
    y = p2[1] - p1[1]
    return np.linalg.norm([x, y])


def ccw(a, b, c):
    """
    Returns positive for cw, negative for ccw, and 0 for collinear.
    Reference: https://www.geeksforgeeks.org/orientation-3-ordered-points/
    """
    return (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])


def line_line_intersection(line1, line2):
    """
    Determines whether the two given line segments intersect. Note this algorithm does not catch the case when both line
    segments are collinear.
    Reference: http://www.jeffreythompson.org/collision-detection/line-rect.php
    :param line1, line2: Line segments in the form ((x1, y1), (x2, y2))
    :return: Point of intersection of the two line segments as tuple(x, y) or None if no intersection
    """
    ((x1, y1), (x2, y2)) = line1
    ((x3, y3), (x4, y4)) = line2

    den = ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
    if den == 0:
        return None  # This means the line segments are parallel or collinear
    u_a = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / den
    u_b = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / den

    # If u_a and u_b are between 0-1, lines are colliding
    if 0 <= u_a <= 1 and 0 <= u_b <= 1:
        intersection_x = x1 + (u_a * (x2-x1))
        intersection_y = y1 + (u_a * (y2-y1))
        return intersection_x, intersection_y
    else:
        return None


def line_rect_intersection(line, rect):
    """
    Determines whether the given line segment and rectangle intersect
    Reference: http://www.jeffreythompson.org/collision-detection/line-rect.php
    :param line: Line segment in the form ((min_x, min_y), (max_x, max_y))
    :param rect: Rectangle in the form ((min_x, min_y), (max_x, max_y))
    :return: List of either 0, 1, or 2 points of intersection
    """
    (r_min_x, r_min_y), (r_max_x, r_max_y) = rect
    rx = r_min_x
    ry = r_min_y
    rw = r_max_x - r_min_x
    rh = r_max_y - r_min_y
    (x1, y1), (x2, y2) = line
    left = line_line_intersection(((x1, y1), (x2, y2)), ((rx, ry), (rx, ry + rh)))
    right = line_line_intersection(((x1, y1), (x2, y2)), ((rx + rw, ry), (rx + rw, ry + rh)))
    top = line_line_intersection(((x1, y1), (x2, y2)), ((rx, ry), (rx + rw, ry)))
    bottom = line_line_intersection(((x1, y1), (x2, y2)), ((rx, ry + rh), (rx + rw, ry + rh)))
    return [p for p in [left, right, top, bottom] if p is not None]


class Polygon:
    def __init__(self, vertices):
        """
        Assumes the vertices are arranged in counterclockwise order
        """
        self.vertices = vertices
        self.center = np.average(self.vertices, axis=0)
        self.convex = self.is_convex()
        self.bounding_box = bounding_box(self.vertices)

    def is_convex(self):
        """
        Walks through the vertices making sure there are no right turns
        """
        for i in range(len(self.vertices)):
            # get consecutive point pairs
            p1 = self.vertices[i - 2]
            p2 = self.vertices[i - 1]
            p3 = self.vertices[i]
            # if points not ccw, return false
            if ccw(p1, p2, p3) > 0:
                return False
        return True

    def point_in_convex_polygon(self, point):
        if not self.convex:
            raise ValueError("only pass convex shapes")

        # do bounding box check first to save time
        x = point[0]
        y = point[1]
        (min_x, min_y), (max_x, max_y) = self.bounding_box
        if not (min_x <= x <= max_x and min_y <= y <= max_y):
            return False

        for i in range(len(self.vertices)):
            # get consecutive point pairs
            p1 = self.vertices[i - 1]
            p2 = self.vertices[i]
            # if points not ccw, return false
            if ccw(p1, p2, point) > 0:
                return False
        return True

    def scale(self, factor):
        self.vertices -= self.center
        self.vertices *= factor
        self.vertices += self.center
