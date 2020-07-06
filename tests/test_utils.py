#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np


def sign(value):
    if value > 0:
        return +1
    elif value < 0:
        return -1
    return 0


def make_square_vertices(side_length=2, center=(0,0)):
    """
    Simple helper function that returns 4 points in counterclockwise order that form a square with sides of length s
    centered at (0,0)
    """
    s = side_length / 2
    x = center[0]
    y = center[1]
    vertices = [[-s+x, -s+y], [s+x, -s+y], [s+x, s+y], [-s+x, s+y]]

    return vertices


def make_circular_vertices(radius=2, center=(0,0), num_pts=8):
    """
    Simple helper function that returns points in counterclockwise order that form a circle with the given radius,
    center, and number of points.
    """
    points = []
    for i in range(num_pts):
        theta = (i / num_pts) * 2*np.pi
        x = radius * np.cos(theta) + center[0]
        y = radius * np.sin(theta) + center[1]
        points.append([x, y])

    return points


def make_linear_vertices(start, end, num_pts=3):
    """
    Helper function that returns points in a straight line from start to end.
    :param start: 2x1 array-like
    :param end: 2x1 array-like
    :param num_pts: Integer (must be >=2)
    """
    start = np.array(start)
    end = np.array(end)
    points = []
    step = (end - start) / num_pts
    for i in range(num_pts):
        points.append(start + i*step)

    return points
