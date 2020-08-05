#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
from collections import defaultdict
import geometry as geom


class Perception:
    """
    Runs full perception stack on a sweep of points from the LIDAR
    """
    def __init__(self, config):
        self.field = config.outer_wall
        self.field_elements = config.field_elements
        self.ball_radius = config.ball_radius

        self.field.scale(0.99)
        for field_element in self.field_elements:
            field_element.scale(1.01)

        self.curr_sweep = []  # Sweep to keep using until sweep builder builds a full one
        self.partial_sweep = []  # Sweep currently being built by sweep builder

    def run(self, vehicle_state):
        """
        :param vehicle_state: Ordered list of dict containing sensor states on the vehicle
        :return: Our pose and list of all obstacles
        """
        # Segmentation
        self.subtract_background(vehicle_state)
        self.cluster(vehicle_state)

        # Classification
        self.classify(vehicle_state)

        world_state = {
            'pose': ((vehicle_state['x'], vehicle_state['y']), vehicle_state['theta']),
            'balls': vehicle_state['balls'],
            'obstacles': vehicle_state['obstacles'],
            'numIngestedBalls': vehicle_state['numIngestedBalls']
        }

        return world_state

    def subtract_background(self, vehicle_state):
        """
        Takes in the sweep stored in vehicle_state['lidarSweepBuilt'] as an Nx2 and creates a mask as an Nx1 array,
        where each element is marked as either 0 for background point or 1 for foreground point. The result is stored
        into vehicle_state['lidarSweepBuiltMask']

        Removes the points in the sweep corresponding to walls and other static obstacles
        :param vehicle_state: Current sweep in world frame as an Nx2 array of floats
        :return: Same as input but with points corresponding to static objects removed
        """
        world_frame_sweep = vehicle_state['lidarSweep']
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
        Identifies clusters of foreground points in the sweep stored in vehicle_state['lidarSweep'] as an Nx2 numpy
        array using the background mask stored in vehicle_state['lidarSweepMask'] as an Nx1 numpy array and marks each
        foreground point with its associated cluster. The result is stored into
        vehicle_state['clusters'].
        """

        # side length of buckets in meters
        BIN_SIZE = 0.1

        sweep = vehicle_state['lidarSweep']
        foreground_points = sweep[vehicle_state['lidarSweepMask']]  # Apply foreground mask to sweep

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

        clusters = geom.connected_components(buckets)
        vehicle_state['clusters'] = [cluster for cluster in clusters if len(cluster) >= 3]

    def classify(self, vehicle_state):
        """
        Classifies each cluster of points as either ball or obstacle from vehicle_state['clusters'], which is an Nx2
        array of points. The resulting balls are stored into vehicle_state['balls'] as a list of circles as
        ((x, y), radius) and the resulting obstacles are placed into vehicle_state['obstacles'] as a list of rectangles
        as ((min_x, min_y), (max_x, max_y))
        """
        clusters = vehicle_state['clusters']
        balls = list()
        obstacles = list()

        for cluster in clusters:
            ball = geom.ransac_circle_fit(cluster, desired_radius=self.ball_radius, consensus=0.99, tolerance=0.03, iterations=10)
            if ball is not None:
                ball_pos, ball_radius = ball
                balls.append(ball_pos)
            else:
                # Construct a bounding box and put into obstacles list
                obstacles.append(geom.bounding_box(cluster))

        vehicle_state['balls'] = balls
        vehicle_state['obstacles'] = obstacles
