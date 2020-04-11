#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np


class Planning:
    def __init__(self):
        self.waypoints = np.array([[0, 0], [1, 0], [1, 1]])

    def run(self, pose, obstacles):
        return self.waypoints
