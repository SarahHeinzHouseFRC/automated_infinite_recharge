#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
from collections import defaultdict
import geometry as geom

IN_TO_M = 0.0254


class Perception:
    """
    Runs full perception stack on a sweep of points from the LIDAR
    """
    def __init__(self, config):
        self.field = config.outer_wall
        self.field_elements = config.field_elements

        self.field.scale(0.99)
        for field_element in self.field_elements:
            field_element.scale(1.01)

    def run(self, vehicle_state):
        """
        :param vehicle_state: Current state of all sensors on the vehicle
        :return: Our pose and list of all obstacles
        """
        # 1. Preprocess sweep
        self.filter_empty_rays(vehicle_state)
        self.parse_sweep(vehicle_state)
        self.spherical_to_cartesian(vehicle_state)

        # 2. Localization
        self.localize(vehicle_state)
        self.vehicle_frame_to_world_frame(vehicle_state)

        # 3. Segmentation
        self.subtract_background(vehicle_state)
        self.cluster(vehicle_state)

        # 4. Classification
        self.classify(vehicle_state)

        """
        Return pose as ((x, y), theta)
        Return obstacles as
        {
            'balls': list(),
            'others': list()
        }
        """
        world_state = {
            'pose': ((vehicle_state['x'], vehicle_state['y']), vehicle_state['theta']),
            'obstacles': vehicle_state['classes'],
            'ingestedBalls': vehicle_state['ingestedBalls']
        }

        return world_state

    def filter_empty_rays(self, vehicle_state):
        """
        Filters the sweep contained in vehicle_state['lidarSweep'] as a list of dict to remove all rays with a negative
        range, which represents a ray miss. The result is stored into vehicle_state['lidarSweepFiltered'] as a smaller
        list of dict.
        """
        sweep = vehicle_state['lidarSweep']
        filtered_sweep = list()
        for point in sweep:
            if point['range'] > 0:
                filtered_sweep.append(point)

        vehicle_state['lidarSweepFiltered'] = filtered_sweep

    def parse_sweep(self, vehicle_state):
        """
        Converts the sweep stored in vehicle_state['lidarSweepFiltered'] from a list of dict with fields 'azimuth',
        'elevation', and 'range' to an Nx3 numpy array. Result is stored into vehicle_state['lidarSweepParsed'].
        """
        sweep = vehicle_state['lidarSweepFiltered']
        num_pts = len(sweep)
        parsed = np.empty((num_pts, 3))
        for i in range(num_pts):
            parsed[i][0] = sweep[i]['azimuth']
            parsed[i][1] = sweep[i]['elevation']
            parsed[i][2] = sweep[i]['range']
        vehicle_state['lidarSweepParsed'] = parsed

    def spherical_to_cartesian(self, vehicle_state):
        """
        Converts the sweep stored in vehicle_state['lidarSweepParsed'] from an Nx3 numpy array of spherical coordinates
        to an Nx2 array of Cartesian coordinates. Result is stored into vehicle_state['lidarSweepCartesian'].
        """
        spherical_sweep = vehicle_state['lidarSweepParsed']
        azimuths = spherical_sweep[:, 0]
        elevations = spherical_sweep[:, 1]
        ranges = spherical_sweep[:, 2]

        cartesian_sweep = np.empty((len(spherical_sweep), 2))
        cartesian_sweep[:, 0] = ranges * np.cos(azimuths)
        cartesian_sweep[:, 1] = ranges * np.sin(azimuths)

        vehicle_state['lidarSweepCartesian'] = cartesian_sweep

    def localize(self, vehicle_state):
        """
        Takes in the sweep stored in vehicle_state['lidarSweepCartesian'] and determines our current position in the
        field. The result is stored into vehicle_state['x'], ['y'], and ['theta']
        :return: Vehicle's current pose as a dict and sweep in world frame as Nx3 numpy array of floats
        """
        pass  # TODO: Cheating for now, the sim has already provided us with x, y, theta

    def vehicle_frame_to_world_frame(self, vehicle_state):
        """
        Converts the sweep stored in vehicle_state['lidarSweepCartesian'] as an Nx2 numpy array from vehicle frame to
        world frame using vehicle_state['x'], ['y'], and ['theta']. Result is stored into
        vehicle_state['lidarSweepWorld'] as an Nx2 numpy array.
        """
        theta = -vehicle_state['theta']
        x = vehicle_state['x']
        y = vehicle_state['y']

        r = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        t = np.array([x, y])

        sweep_vehicle_frame = vehicle_state['lidarSweepCartesian']
        sweep_world_frame = (sweep_vehicle_frame @ r) + t
        vehicle_state['lidarSweepWorld'] = sweep_world_frame

    def subtract_background(self, vehicle_state):
        """
        Takes in the sweep stored in vehicle_state['lidarSweepWorld'] as an Nx2 and creates a mask as an Nx1 array,
        where each element is marked as either 0 for background point or 1 for foreground point. The result is stored
        into vehicle_state['lidarSweepMask']

        Removes the points in the sweep corresponding to walls and other static obstacles
        :param vehicle_state: Current sweep in world frame as an Nx2 array of floats
        :return: Same as input but with points corresponding to static objects removed
        """
        world_frame_sweep = vehicle_state['lidarSweepWorld']
        background_mask = np.zeros(len(world_frame_sweep), dtype=bool)

        for i, point in enumerate(world_frame_sweep):
            if (self.field.point_in_convex_polygon(point) and not any(
                    fe.point_in_convex_polygon(point) for fe in self.field_elements)):
                background_mask[i] = True

            # if self.field.point_in_convex_polygon(point)
            # and not self.bottom_column.point_in_convex_polygon(point)
            # and not self.bottom_column.point_in_convex_polygon(point)
            # and not self.bottom_column.point_in_convex_polygon(point)
            # and not self.bottom_column.point_in_convex_polygon(point):
            #     # background_subtracted_sweep.append(point)
            #     background_mask[i] = 1

        vehicle_state['lidarSweepMask'] = background_mask

    def cluster(self, vehicle_state):
        """
        Identifies clusters of foreground points in the sweep stored in vehicle_state['lidarSweepWorld'] as an Nx2 numpy
        array using the background mask stored in vehicle_state['lidarSweepMask'] as an Nx1 numpy array and marks each
        foreground point with its associated cluster. The result is stored into
        vehicle_state['clusters'].
        """

        # side length of buckets in meters
        BIN_SIZE = 0.1

        worldSweep = vehicle_state['lidarSweepWorld']
        foreground_points = worldSweep[vehicle_state['lidarSweepMask']]

        # Bucketize all points into a defaultdict of this form:
        # buckets = {
        #     (0,0) : list(... points ...),
        #     (1,0) : list(... points ...),
        #     (2,0) : list(... points ...),
        # }
        buckets = defaultdict(list)
        for point in foreground_points:
            bucket = (point[0] // BIN_SIZE, point[1] // BIN_SIZE) # get coordinates from point
            buckets[bucket].append(point)

        vehicle_state['clusters'] = geom.connected_components(buckets)

    def classify(self, vehicle_state):
        """
        Classifies each cluster of points as either GAMEPIECE or ROBOT from vehicle_state['clusters'], which is an Nx2
        array of points. The result is stored into vehicle_state['classes'] as an Nx1 array of ints, 1 for GAMEPIECE
        or 2 for ROBOT.
        """
        clusters = vehicle_state['clusters']
        balls = list()
        others = list()

        for cluster in clusters:
            circle = geom.ransac_circle_fit(cluster, consensus=0.99, tolerance=0.03, iterations=10)
            if circle is not None and 3.45*IN_TO_M <= circle[1] <= 3.55*IN_TO_M: # Balls are 3.5" in radius
                balls.append(circle)
            else:
                # Construct a bounding box and put into others list
                others.append(geom.bounding_box(cluster))

        vehicle_state['classes'] = {
            'balls': balls,
            'others': others
        }
