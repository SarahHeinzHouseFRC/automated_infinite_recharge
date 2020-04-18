#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np


class Planning:
    def __init__(self):
        self.waypoint = np.array([2, 2])

    def run(self, pose, obstacles):
        return self.waypoint
