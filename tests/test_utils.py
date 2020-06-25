#
# Copyright (c) 2020 FRC Team 3260
#


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
