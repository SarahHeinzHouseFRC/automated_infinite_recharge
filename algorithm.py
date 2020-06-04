#
# Copyright (c) 2020 FRC Team 3260
#

from math import sqrt
import numpy as np
from collections import defaultdict
import matplotlib.pyplot as plt


class Node:
    def __init__(self, position):
        """
        :param position: Node's position as tuple(x, y)
        """
        self.position = position
        self.neighbors = set()
        self.parent = None
        self.occupied = False

    def __lt__(self, other):
        return self.position < other.position


class Grid:
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

        # 1. Create all the nodes
        for col in range(self.num_cols):
            for row in range(self.num_rows):
                x = (col * self.cell_resolution) + (self.cell_resolution / 2) - self.width / 2
                y = (row * self.cell_resolution) + (self.cell_resolution / 2) - self.height / 2
                self.grid[col][row] = Node((x, y))

        # 2. Connect all the neighbors
        min_col = 0
        max_col = self.num_cols-1
        min_row = 0
        max_row = self.num_rows-1
        for x in range(self.num_cols):
            for y in range(self.num_rows):
                node = self.grid[x][y]
                if x > min_col:
                    node.neighbors.add(self.grid[x-1][y])
                if x < max_col-1:
                    node.neighbors.add(self.grid[x+1][y])
                if y > min_row:
                    node.neighbors.add(self.grid[x][y-1])
                if y < max_row-1:
                    node.neighbors.add(self.grid[x][y+1])

    def insert_obstacles(self, obstacles):
        """
        Find the grid cells corresponding to each obstacle and marks them as occupied.
        :param obstacles: List of rectangular obstacles where each obstacle is ((min_x, min_y), (max_x, max_y))
        """
        for ((min_x, min_y), (max_x, max_y)) in obstacles:
            min_col = int((min_x + self.origin[0]) // self.cell_resolution)
            max_col = int((max_x + self.origin[0]) // self.cell_resolution)
            min_row = int((min_y + self.origin[1]) // self.cell_resolution)
            max_row = int((max_y + self.origin[1]) // self.cell_resolution)

            for col in range(min_col, max_col):
                for row in range(min_row, max_row):
                    self.grid[col][row].occupied = True

    def clear(self):
        """
        Marks all nodes as unoccupied
        """
        for row in self.grid:
            for cell in row:
                cell.occupied = False

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


def a_star(grid, start, goal):
    """
    Returns a trajectory from start to goal as a list of Nodes or None if no path is found. The returned trajectory will
    start with the start node and end with the goal node
    :param start: Starting node
    :param goal: Goal node
    :param grid: Grid object
    :return: List of trajectory points as list(tuple(x, y), ...)
    """
    queue = [(0, start)]

    while len(queue) > 0 and goal.parent is None:
        # Pop a node from the queue
        _, cur_node = queue.pop(0)

        # Run through its neighbors
        for neighbor in cur_node.neighbors:
            # If unvisited, add it to the queue
            if neighbor.parent is None:
                neighbor.parent = cur_node
                cost = grid.cell_resolution # How much does it cost to get to this neighbor from cur_node?
                heuristic = dist(neighbor.position, goal.position) # How close is this cell to the goal?
                score = heuristic + cost
                queue.append((score, neighbor))
                queue.sort()
            if neighbor == goal:
                break

    path = []
    cur_node = goal

    # If no path was found, return none
    if goal.parent is None:
        return None

    # Iterate backwards from goal
    while cur_node != start:
        path.insert(0, cur_node)
        cur_node = cur_node.parent

    # Add starting node to path
    path.insert(0, start)

    return path


def bounding_box(points):
    """
    Takes a list of points and returns a tuple (min_x, max_x, min_y, max_y)
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
        return [(self.x-1,self.y),
                (self.x+1,self.y),
                (self.x,self.y-1),
                (self.x,self.y+1)]


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


def ransac_circle_fit(points, consensus, tolerance, iterations):
    """
    Takes N points as an Nx2 array and returns the coordinates of a best-fit circle.
    :param points: Nx2 numpy array of points
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
        fit_circle = make_circle(points[random_sample[0]], points[random_sample[1]], points[random_sample[2]])

        # 3. Check all other points in the cluster for consensus
        fit_circle_center = fit_circle[0]
        fit_circle_radius = fit_circle[1]
        num_inliers = 0
        for point in points:
            if abs(dist(point, fit_circle_center) - fit_circle_radius) <= tolerance:
                num_inliers += 1

        # 4. If we've reached a consensus, then return the circle we've found
        if num_inliers / len(points) >= consensus:
            return fit_circle
    return None


def dist(p1, p2):
    """
    Takes 2 points as array-like and returns the Euclidean distance between them.
    """
    x = p2[0] - p1[0]
    y = p2[1] - p1[1]
    return np.linalg.norm([x, y])


def make_circle(p1, p2, p3):
    """
    Constructs a circle from 3 points.
    Source: https://www.w3resource.com/python-exercises/basic/python-basic-1-exercise-39.php
    :param p1, p2, p3: Three 2D points as numpy arrays of shape (2,)
    :return: Circle as tuple(tuple(x, y), radius)
    """
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


def main():
    buckets = defaultdict(list)
    buckets[(0,0)] = [[0,0], [0,0], [0,0], [0,0]]
    buckets[(1,0)] = [[1,1], [1,1], [1,1], [1,1]]
    buckets[(1,1)] = [[2,2], [2,2], [2,2], [2,2]]
    buckets[(10,10)] = [[3,3], [3,3], [3,3], [3,3]]
    ccs = connected_components(buckets)
    assert len(ccs) == 2, 'Wrong number of connected components found'

    points = np.array([[-1.11, -1.60], [-1.11, -1.64], [-1.15, -1.70]])
    circle = make_circle(points[0], points[1], points[2])
    print(circle)
    if circle is not None:
        circle_artist = plt.Circle((circle[0], circle[1]), circle[2], color=(1.0, 0.37, 0.22, 1.0))
        fig, ax = plt.subplots()
        ax.add_artist(circle_artist)
    plt.plot(points[:,0], points[:,1], marker='o', linestyle='')
    plt.axis('equal')
    plt.show()


if __name__ == '__main__':
    main()
