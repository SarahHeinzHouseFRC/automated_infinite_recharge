#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import algorithm as alg
from collections import defaultdict
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
        for field_element in self.field_elements:
            field_element.scale(1.01)
        self.field = Polygon(self.outer_wall)
        self.field.scale(0.99)

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

        # 5. Postprocess sweep
        self.postprocess_objects(vehicle_state)

        self.visualize(vehicle_state)

        """
        Return pose as
        {
            'x': x,         # Meters
            'y': y,         # Meters
            'theta': theta, # Radians
        }
        Return obstacles as
        {
            'balls': list(),
            'other': list()
        }
        """
        pose = {
            'x': vehicle_state['x'],
            'y': vehicle_state['y'],
            'theta': vehicle_state['theta'],
        }
        obstacles = vehicle_state['classes']

        return pose, obstacles

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

        vehicle_state['clusters'] = alg.connected_components(buckets)

    def classify(self, vehicle_state):
        """
        Classifies each cluster of points as either GAMEPIECE or ROBOT from vehicle_state['clusters'], which is an Nx2
        array of points. The result is stored into vehicle_state['classes'] as an Nx1 array of ints, 1 for GAMEPIECE
        or 2 for ROBOT.
        """
        clusters = vehicle_state['clusters']
        balls = list()
        other = list()

        for cluster in clusters:
            circle = alg.ransac_circle_fit(cluster, consensus=0.99, tolerance=0.03, iterations=50)
            if circle is not None and 3.45*IN_TO_M <= circle[2] <= 3.55*IN_TO_M: # Balls are 3.5" in radius
                balls.append(circle)
            else:
                # Construct a bounding box and put into other list
                other.append(alg.bounding_box(cluster))

        vehicle_state['classes'] = {
            'balls' : balls,
            'others' : other
        }

    def postprocess_objects(self, vehicle_state):
        """
        TBD.
        """
        pass

    def visualize(self, vehicle_state):
        plt.clf()
        fig = plt.gcf()
        ax = fig.gca()

        # Plot vehicle position
        x = vehicle_state['x']
        y = vehicle_state['y']
        vehicle_position = np.array([x, y])
        plt.plot(vehicle_position[0], vehicle_position[1], color=(1.0, 0.37, 0.22, 1.0), marker='x', linestyle='')
        plt.text(vehicle_state['x'], vehicle_state['y'] + 0.05, "Self", color='r', fontsize=10)

        # Plot background subtraction
        world_frame_sweep = vehicle_state['lidarSweepWorld']
        mask0 = np.array(vehicle_state['lidarSweepMask'], dtype=bool)
        mask1 = np.invert(mask0)
        plt.scatter(world_frame_sweep[:, 0][mask0], world_frame_sweep[:, 1][mask0], marker='.', color=(0.15, 0.65, 0.65, 1.0), label='Foreground point')
        plt.scatter(world_frame_sweep[:, 0][mask1], world_frame_sweep[:, 1][mask1], marker='.', color='gray', label='Background point')

        # Plot segmentation
        # plt.fill(self.field.vertices[:, 0], self.field.vertices[:, 1], fc=(0,0,0,0), ec=(0.15, 0.65, 0.65, 0.8))
        # for poly in self.field_elements:
        #     plt.fill(poly.vertices[:,0], poly.vertices[:,1], fc=(0.15, 0.65, 0.65, 0.8))

        # Plot clusters
        # i = 0
        # for cluster in vehicle_state['clusters']:
        #     plt.scatter(cluster[:, 0], cluster[:, 1], marker='.', label=i)
        #     i += 1

        # Plot the goal
        min_dist = 1e5
        for ball in vehicle_state['classes']['balls']:
            curr_dist = alg.distance(np.array([ball[0], ball[1]]), np.array([x, y]))
            if curr_dist < min_dist:
                goal_x, goal_y = np.array([ball[0]-0.25, ball[1]-0.25])
        plt.text(goal_x, goal_y + 0.5 + 0.05, "Goal", color='g', fontsize=10)
        bbox = patches.Rectangle((goal_x, goal_y), 0.5, 0.5, linewidth=1, edgecolor='g', facecolor='none')
        ax.add_patch(bbox)

        # Plot classes
        for c in vehicle_state['classes']['balls']:
            if c is not None:
                x = c[0]-c[2] - 0.05
                y = c[1]-c[2] - 0.05
                width = 2*c[2] + 0.1
                height = 2*c[2] + 0.1
                plt.text(x, y+height+0.05, "Ball", color='r', fontsize=10)
                # circle = plt.Circle((c[0], c[1]), c[2], color=(0.75, 0.75, 0.0, 0.5))
                # ax.add_patch(circle)
                bbox = patches.Rectangle((x, y), width, height, linewidth=1, edgecolor='r', facecolor='none')
                ax.add_patch(bbox)

        plt.title("Team SHARP FRC 2020 Perception Stack")
        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.axis('equal')
        plt.legend(loc='lower right')
        plt.draw()
        plt.pause(0.1)


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
