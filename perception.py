#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
import matplotlib.pyplot as plt
from test_data import TestData


class Perception:
    """
    Runs full perception stack on a full sweep of points from the LIDAR
    """
    def __init__(self):
        self.localization = Localization()
        self.background_subtraction = BackgroundSubtraction()
        self.clustering = Clustering()

    def run(self, sweep):
        """
        :param sweep: Latest sweep from the LIDAR as an Nx3 array of floats
        :return: Our pose and list of all obstacles
        """
        pose, world_frame_sweep = self.localization.run(sweep)
        background_subtracted_sweep = self.background_subtraction.run(world_frame_sweep)
        obstacles = self.clustering.run(background_subtracted_sweep)
        return pose, obstacles


class Localization:
    """
    Takes in the current sweep and determines our current position in the field by comparing it to the nominal sweep
    """
    def __init__(self):
        test_data = TestData()
        self.nominal_sweep = test_data.nominal_input.sweep

    def run(self, sweep):
        """
        :param sweep: Nx3 numpy array of floats in vehicle frame
        :return: Vehicle's current pose as a dict and sweep in world frame as Nx3 numpy array of floats
        """
        pose = {
            "x": 0,
            "y": 0,
            "theta": 0,
        }
        return pose, sweep


class BackgroundSubtraction:
    """
    Removes the points in the sweep corresponding to walls and other static obstacles
    """
    def __init__(self):
        pass

    def run(self, world_frame_sweep):
        """
        :param world_frame_sweep: Current sweep in world frame as an Nx3 array of floats
        :return: Same as input but with points corresponding to static objects removed
        """
        pass


class Clustering:
    """
    Clumps together clusters of points and forms a list of obstacles
    """
    def __init__(self):
        pass

    def run(self, background_subtracted_sweep):
        """
        :param background_subtracted_sweep: Background-subtracted sweep as Nx3 array of floats
        :return: List of detected obstacles, their positions, and radii
        """
        pass


def main():
    test_data = TestData()
    perception = Perception()
    perception.run(test_data.nominal_input.sweep)


if __name__ == '__main__':
    main()