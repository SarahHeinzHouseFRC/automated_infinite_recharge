#
# Copyright (c) 2020 FRC Team 3260
#

import matplotlib.pyplot as plt
import numpy as np


def ccw(a, b, c):
    """
    Returns positive for cw, negative for ccw, and 0 for collinear.
    Reference: https://www.geeksforgeeks.org/orientation-3-ordered-points/
    """
    return (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])


class Polygon:
    def __init__(self, vertices):
        self.vertices = vertices
        self.center = np.average(self.vertices, axis=0)

    def point_in_convex_polygon(self, point):
        for i in range(len(self.vertices)):
            # get consecutice point paris
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


def main():
    points = np.array([[0, 0], [0, 10], [15, 0], [-15, 0], [0, 25], [0, -15]])
    poly = Polygon(np.array([[10, -10], [10, 10], [0, 20], [-10, 10], [-10, -10]]))

    for point in points:
        print(poly.point_in_convex_polygon(point))  # True, True, False, False, False, False

    # Display the polygon and points
    plt.fill(poly.vertices[:,0], poly.vertices[:,1], facecolor=(0.15, 0.65, 0.65, 0.2), edgecolor=(0.15, 0.65, 0.65, 1.0), linewidth=1)
    plt.plot(points[:,0], points[:,1], color=(1.0, 0.37, 0.22, 1.0), marker='o', linestyle='')
    plt.axis('equal')
    plt.title('Point in Polygon Test')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.show()


if __name__ == '__main__':
    main()
