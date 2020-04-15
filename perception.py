#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
import numpy.ma as ma
import matplotlib.pyplot as plt
from geometry import Polygon

IN_TO_M = 0.0254


class Perception:
    """
    Runs full perception stack on a sweep of points from the LIDAR
    """
    def __init__(self):
        plt.ion()

        self.outer_wall = IN_TO_M * np.array([
            [161.81, 288.58],
            [90.94, 314.96],
            [-90.94, 314.96],
            [-161.81, 288.58],
            [-161.81, -288.58],
            [-90.94, -314.96],
            [90.94, -314.96],
            [161.81, -288.58],
        ])
        self.right_column = IN_TO_M * np.array([
            [105.12, -47.64],
            [100.39, -36.61],
            [89.37, -41.34],
            [94.09, -52.36],
        ])
        self.top_column = IN_TO_M * np.array([
            [44.88, 97.64],
            [40.55, 108.27],
            [29.53, 103.54],
            [33.86, 92.52],
        ])
        self.left_column = IN_TO_M * np.array([
            [-89.37, 41.34],
            [-94.09, 52.36],
            [-105.12, 47.64],
            [-100.39, 36.61],
        ])
        self.bottom_column = IN_TO_M * np.array([
            [-29.53, -103.54],
            [-33.86, -92.52],
            [-44.88, -96.85],
            [-40.55, -108.27],
        ])
        self.right_trench_right_wall = IN_TO_M * np.array([
            [161.81, -59.84],
            [161.81, -29.92],
            [159.84, -29.92],
            [159.84, -59.84],
        ])
        self.right_trench_left_wall = IN_TO_M * np.array([
            [107.48, -59.84],
            [107.48, -29.92],
            [105.51, -29.92],
            [105.51, -59.84],
        ])
        self.left_trench_right_wall = IN_TO_M * np.array([
            [-159.84, 29.92],
            [-159.84, 59.84],
            [-161.81, 59.84],
            [-161.81, 29.92],
        ])
        self.left_trench_left_wall = IN_TO_M * np.array([
            [-105.51, 29.92],
            [-105.51, 59.84],
            [-107.48, 59.84],
            [-107.48, 29.92],
        ])
        self.field_elements = [Polygon(_) for _ in
                               [self.right_column, self.left_column, self.top_column, self.bottom_column,
                                self.right_trench_right_wall, self.right_trench_left_wall,
                                self.left_trench_right_wall, self.left_trench_left_wall]]
        for fe in self.field_elements:
            fe.scale(1.01)
        self.field = Polygon(self.outer_wall)
        self.field.scale(0.99)

    def run(self, vehicle_state):
        """
        :param vehicle_state: Current state of all sensors on the vehicle
        :return: Our pose and list of all obstacles
        """
        vehicle_state['lidarSweepCartesian'] = self.spherical_to_cartesian(self.parse_sweep(vehicle_state['lidarSweep']))
        pose, world_frame_sweep = self.localize(vehicle_state)
        background_mask = self.subtract_background(world_frame_sweep)

        mask0 = np.array(background_mask, dtype=bool)
        mask1 = np.invert(mask0)

        vehicle_position = np.array([[pose['x'], pose['y']],])

        plt.clf()
        plt.scatter(world_frame_sweep[:, 0][mask0], world_frame_sweep[:, 1][mask0], c='red', marker='.', label='Foreground point')
        plt.scatter(world_frame_sweep[:, 0][mask1], world_frame_sweep[:, 1][mask1], c='gray', marker='.', label='Background point')
        plt.plot(vehicle_position[:, 0], vehicle_position[:, 1], color=(1.0, 0.37, 0.22, 1.0), marker='x', linestyle='')
        plt.legend(loc='lower right')
        # plt.fill(self.field.vertices[:, 0], self.field.vertices[:, 1], fc=(0,0,0,0), ec=(1.0, 0.37, 0.22, 0.5))
        # for poly in self.field_elements:
        #     plt.fill(poly.vertices[:,0], poly.vertices[:,1], fc=(1.0, 0.37, 0.22, 0.5))
        plt.title("LIDAR Sweep Segmentation")
        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.axis('equal')
        plt.draw()
        plt.pause(0.1)

        obstacles = self.cluster(world_frame_sweep, background_mask)
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
        :param world_frame_sweep: Current sweep in world frame as an Nx2 array of floats
        :return: Same as input but with points corresponding to static objects removed
        """

        background_mask = np.zeros(len(world_frame_sweep), dtype=int)

        for i, point in enumerate(world_frame_sweep):
            if (self.field.point_in_convex_polygon(point) and not any(
                    [fe.point_in_convex_polygon(point) for fe in self.field_elements])):
                background_mask[i] = 1

            # if self.field.point_in_convex_polygon(point)
            # and not self.bottom_column.point_in_convex_polygon(point)
            # and not self.bottom_column.point_in_convex_polygon(point)
            # and not self.bottom_column.point_in_convex_polygon(point)
            # and not self.bottom_column.point_in_convex_polygon(point):
            #     # background_subtracted_sweep.append(point)
            #     background_mask[i] = 1

        return background_mask

    def cluster(self, world_frame_sweep, background_mask):
        """
        Clumps together clusters of points and forms a list of obstacles
        :param world_frame_sweep: Sweep in world frame as Nx2 array of floats
        :param background_mask:
        :return: List of detected obstacles, their positions, and radii
        """
        pass


_ = '''
if self.field.point_in_convex_polygon(point)
    and not any([col.point_in_convex_polygon(point) for col in self.field_elements]:
'''

_ = '''
import numpy as np
import numpy.ma as ma
arr1 = np.array([0,1,2,3,4,5])
mask = np.array([0,1,0,1,0,1])
ma.masked_array(arr1, mask)
## masked_array(data=[0, --, 2, --, 4, --],
##             mask=[False,  True, False,  True, False,  True],
##       fill_value=999999)
'''
