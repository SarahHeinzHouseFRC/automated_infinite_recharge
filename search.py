#
# Copyright (c) 2020 FRC Team 3260
#

from math import sqrt
import numpy as np
from collections import defaultdict
import matplotlib.pyplot as plt


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


def ransac(points, precision):
    """
    Takes points as an Nx2 array and returns whether or not the points fit a circle and the coordinates of the circle.
    Result: tuple(x, y, radius) or None
    """
    for _ in range(20):
        # 1. Take a random sample of 3 points from this cluster
        if(len(points) >= 3):
            subset = np.random.choice(len(points), 3, replace=False)
        else:
            return None
        # 2. Fit a circle to those points
        fit_circle = make_circle(points.take(subset, axis=0))

        # 3. Check all other points in the cluster for consensus
        inliers = 0
        for point in points:
            if abs(7 - distance(point, fit_circle[0:2]) < 0.5):
                inliers += 1
        # 4. If we have a consensus (maybe 90%), then return out
        if inliers/len(points) >= precision:
            return fit_circle
    return None


def distance(point1, point2):
    """
    Takes 2 points as Nx2 arrays and returns the distance between them
    """
    return np.linalg.norm(point1 - point2)

    #without numpy
    # x_dis = abs(point1[0] - point2[0])
    # y_dis = abs(point1[1] - point2[1])
    # return sqrt((x_dis ** 2) + (y_dis ** 2))

def make_circle(points):
    """
    Constructs a circle from 3 points. Takes a 3x2 numpy array of points and returns a tuple(x, y, radius).
    """
    x1 = points[0,0]
    x2 = points[1,0]
    x3 = points[2,0]
    y1 = points[0,1]
    y2 = points[1,1]
    y3 = points[2,1]

    x12 = x1 - x2
    x13 = x1 - x3

    y12 = y1 - y2
    y13 = y1 - y3

    y31 = y3 - y1
    y21 = y2 - y1

    x31 = x3 - x1
    x21 = x2 - x1

    # x1^2 - x3^2
    sx13 = pow(x1, 2) - pow(x3, 2)

    # y1^2 - y3^2
    sy13 = pow(y1, 2) - pow(y3, 2)

    sx21 = pow(x2, 2) - pow(x1, 2)
    sy21 = pow(y2, 2) - pow(y1, 2)

    f = (((sx13) * (x12) + (sy13) *
          (x12) + (sx21) * (x13) +
          (sy21) * (x13)) // (2 *
          ((y31) * (x12) - (y21) * (x13))))

    g = (((sx13) * (y12) + (sy13) * (y12) +
          (sx21) * (y13) + (sy21) * (y13)) //
          (2 * ((x31) * (y12) - (x21) * (y13))))

    c = (-pow(x1, 2) - pow(y1, 2) -
         2 * g * x1 - 2 * f * y1)

    # eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0
    # where centre is (h = -g, k = -f) and
    # radius r as r^2 = h^2 + k^2 - c
    h = -g
    k = -f
    sqr_of_r = h * h + k * k - c

    # r is the radius
    r = round(sqrt(sqr_of_r), 5)

    return np.array([h,k,r])


def main():
    buckets = defaultdict(list)
    buckets[(0,0)] = [[0,0], [0,0], [0,0], [0,0]]
    buckets[(1,0)] = [[1,1], [1,1], [1,1], [1,1]]
    buckets[(1,1)] = [[2,2], [2,2], [2,2], [2,2]]
    buckets[(10,10)] = [[3,3], [3,3], [3,3], [3,3]]
    ccs = connected_components(buckets)
    assert len(ccs) == 2, "Wrong number of connected components found"

    points = np.array([[-10,10], [0,20], [10,10]])
    c = make_circle(points)
    circle_artist = plt.Circle((c[0], c[1]), c[2], color=(1.0, 0.37, 0.22, 1.0))
    fig = plt.gcf()
    ax = fig.gca()
    # ax.add_artist(circle_artist)
    plt.plot(points, marker='o', linestyle='')
    plt.show()


if __name__ == '__main__':
    main()
