#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
import algorithm as alg


class Planning:
    def __init__(self):
        self.waypoint = np.array([2, 2])

    def run(self, pose, obstacles):
        # 1. Identify the goal
        start = (pose['x'], pose['y']) # Our current (x,y)
        min_dist = np.inf
        goal = None
        for ball in obstacles['balls']:
            curr_dist = alg.distance(np.array((ball[0],ball[1])), np.array(start))
            if curr_dist < min_dist:
                min_dist = curr_dist
                goal = ball[:2]

        # 2. Move towards it if there is a nearest ball (using A*)

        return goal
