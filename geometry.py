#
# Copyright (c) 2020 FRC Team 3260
#

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y

class Line:
    def __init__(self, a, b):
        self.a = a
        self.b = b

class Polygon:
    def __init__(self, vertices):
        self.vertices = vertices

    def point_in_polygon(self, point):
        pass
