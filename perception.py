#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
import matplotlib.pyplot as plt
from geometry import *


class Perception:
    """
    Runs full perception stack on a sweep of points from the LIDAR
    """
    def __init__(self):
        plt.ion()

        self.outer_wall = np.array([
            [7.33, -4.11],
            [8.0, -2.31],
            [8.0, 2.31],
            [7.33, 4.11],
            [-7.33, 4.11],
            [-8.0, 2.31],
            [-8.0, -2.31],
            [-7.33, -4.11],
        ])
        self.right_column = 0.0254 * np.array([
            [105.12, -47.64],
            [100.39, -36.61],
            [89.37, -41.34],
            [94.09, -52.36],
        ])
        self.top_column = 0.0254 * np.array([
            [44.88, 97.64],
            [40.55, 108.27],
            [29.53, 103.54],
            [33.86, 92.52],
        ])
        self.left_column = 0.0254 * np.array([
            [-89.37, 41.34],
            [-94.09, 52.36],
            [-105.12, 47.64],
            [-100.39, 36.61],
        ])
        self.bottom_column = 0.0264 * np.array([
            [-29.53, -103.54],
            [-33.86, -92.52],
            [-44.88, -96.85],
            [-40.55, -108.27],
        ])

    def run(self, vehicle_state):
        """
        :param vehicle_state: Current state of all sensors on the vehicle
        :return: Our pose and list of all obstacles
        """
        vehicle_state['lidarSweepCartesian'] = self.spherical_to_cartesian(self.parse_sweep(vehicle_state['lidarSweep']))
        pose, world_frame_sweep = self.localize(vehicle_state)

        vehicle_position = np.array([[pose['x'], pose['y']],])

        plt.clf()
        plt.plot(world_frame_sweep[:, 0], world_frame_sweep[:, 1], c=(0.15, 0.65, 0.65, 0.2), marker='.', linestyle='')
        plt.plot(vehicle_position[:, 0], vehicle_position[:, 1], color=(1.0, 0.37, 0.22, 0.5), marker='x', linestyle='')
        plt.draw()
        plt.axis('equal')
        plt.pause(0.1)

        background_subtracted_sweep = self.subtract_background(world_frame_sweep)
        obstacles = self.cluster(background_subtracted_sweep)
        return pose, obstacles

    def parse_sweep(self, sweep):
        """
        Converts a list of dict with fields 'azimuth', 'elevation', and 'range' to numpy array
        :param sweep: List of dict
        :return: Nx3 numpy array
        """
        num_pts = len(sweep)
        parsed = np.empty((num_pts, 3))
        for i in range(num_pts):
            az = sweep[i]['azimuth']
            el = sweep[i]['elevation']
            r = sweep[i]['range']
            parsed[i][0] = az
            parsed[i][1] = el
            parsed[i][2] = r
        return parsed

    def spherical_to_cartesian(self, spherical_sweep):
        """
        Converts a sweep from spherical to Cartesian coordinates
        :param spherical_sweep: Sweep in spherical coordinates Nx3 numpy array
        :return: Sweep in Cartesian coordinates as an Nx2 numpy array
        """
        azimuths = spherical_sweep[:, 0]
        elevations = spherical_sweep[:, 1]
        ranges = spherical_sweep[:, 2]

        cartesian_sweep = np.empty((len(spherical_sweep), 2))
        cartesian_sweep[:, 0] = ranges * np.cos(azimuths)
        cartesian_sweep[:, 1] = ranges * np.sin(azimuths)

        return cartesian_sweep

    def localize(self, vehicle_state):
        """
        Takes in the latest sensor data and determines our current position in the field
        :param vehicle_state: Nx3 numpy array of floats in vehicle frame
        :return: Vehicle's current pose as a dict and sweep in world frame as Nx3 numpy array of floats
        """
        # Determine our current pose
        theta = -vehicle_state['theta']
        x = vehicle_state['x']
        y = vehicle_state['y']

        # Transform the sweep from vehicle frame to global frame
        r = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        t = np.array([x, y])

        sweep_vehicle_frame = vehicle_state['lidarSweepCartesian']
        sweep_world_frame = (sweep_vehicle_frame @ r) + t

        pose = {
            'x': x,
            'y': y,
            'theta': theta,
        }

        return pose, sweep_world_frame

    def subtract_background(self, world_frame_sweep):
        """
        Removes the points in the sweep corresponding to walls and other static obstacles
        :param world_frame_sweep: Current sweep in world frame as an Nx3 array of floats
        :return: Same as input but with points corresponding to static objects removed
        """
        field = Polygon(self.outer_wall)

        background_subtracted_sweep = list()
        for point in world_frame_sweep:
            if field.point_in_convex_polygon(point):
                background_subtracted_sweep.append(point)

        background_subtracted_sweep = np.array(background_subtracted_sweep)

        plt.plot(self.outer_wall[:, 0], self.outer_wall[:, 1], color=(1.0, 0.37, 0.22, 1.0), marker=".", linestyle='')
        plt.plot(self.right_column[:, 0], self.right_column[:, 1], color=(1.0, 0.37, 0.22, 1.0), marker=".", linestyle='')
        plt.plot(self.top_column[:, 0], self.top_column[:, 1], color=(1.0, 0.37, 0.22, 1.0), marker=".", linestyle='')
        plt.plot(self.left_column[:, 0], self.left_column[:, 1], color=(1.0, 0.37, 0.22, 1.0), marker=".", linestyle='')
        plt.plot(self.bottom_column[:, 0], self.bottom_column[:, 1], color=(1.0, 0.37, 0.22, 1.0), marker=".", linestyle='')
        plt.plot(background_subtracted_sweep[:, 0], background_subtracted_sweep[:, 1], color=(0.15, 0.65, 0.65, 1.0), marker=".", linestyle='')

    def cluster(self, background_subtracted_sweep):
        """
        Clumps together clusters of points and forms a list of obstacles
        :param background_subtracted_sweep: Background-subtracted sweep as Nx3 array of floats
        :return: List of detected obstacles, their positions, and radii
        """
        pass
