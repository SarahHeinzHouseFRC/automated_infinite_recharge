#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np


class SweepBuilder:
    def __init__(self):
        self.partial_sweep = []
        self.full_sweep = []
        self.prev_azimuth = None

    def run(self, vehicle_states):
        """
        Takes a list of vehicle states and aggregates them into a single vehicle state. Converts all LIDAR points into
        world frame and returns only full sweeps of points. If a full sweep cannot yet be constructed, returns the last
        full sweep.
        :param vehicle_states: List of latest vehicle states
        :return: A single vehicle state containing a full sweep, or None
        """
        if not vehicle_states:
            return None

        for vehicle_state in vehicle_states:
            self.filter_empty_rays(vehicle_state)
            if len(vehicle_state['lidarPointsFiltered']) == 0:
                continue
            self.spherical_to_cartesian(vehicle_state)
            self.localize(vehicle_state)
            self.vehicle_frame_to_world_frame(vehicle_state)
            self.build_sweep(vehicle_state)

        if len(self.full_sweep) == 0:
            return None

        final_vehicle_state = vehicle_states[-1]
        final_vehicle_state['lidarSweep'] = self.full_sweep
        return final_vehicle_state

    def filter_empty_rays(self, vehicle_state):
        """
        Filters the sweep contained in vehicle_state['lidarPoints'] as an Nx3 array of points to remove all points with
        a negative range, which represent a ray miss rather than a ray hit. The result is stored into
        vehicle_state['lidarPointsFiltered'] as an Mx3 array of points where M <= N.
        """
        vehicle_state['lidarPointsFiltered'] = np.array([point for point in vehicle_state['lidarPoints'] if point[2] > 0])

    def spherical_to_cartesian(self, vehicle_state):
        """
        Converts the sweep stored in vehicle_state['lidarPointsFiltered'] from an Nx3 numpy array of spherical
        coordinates to an Nx2 array of Cartesian coordinates. Result is stored into
        vehicle_state['lidarPointsCartesian'].
        """
        spherical_sweep = vehicle_state['lidarPointsFiltered']
        azimuths = spherical_sweep[:, 0]
        elevations = spherical_sweep[:, 1]  # pylint: disable=unused-variable
        ranges = spherical_sweep[:, 2]

        cartesian_sweep = np.empty((len(spherical_sweep), 2))
        cartesian_sweep[:, 0] = ranges * np.cos(azimuths)
        cartesian_sweep[:, 1] = ranges * np.sin(azimuths)

        vehicle_state['lidarPointsCartesian'] = cartesian_sweep

    def localize(self, vehicle_state):
        """
        Takes in the sweep stored in vehicle_state['lidarPointsCartesian'] and determines our current position in the
        field. The result is stored into vehicle_state['x'], ['y'], and ['theta']
        :return: Vehicle's current pose as a dict and sweep in world frame as Nx3 numpy array of floats
        """
        pass  # TODO: Cheating for now, the sim has already provided us with x, y, theta

    def vehicle_frame_to_world_frame(self, vehicle_state):
        """
        Converts the sweep stored in vehicle_state['lidarPointsCartesian'] as an Nx2 numpy array from vehicle frame to
        world frame using vehicle_state['x'], ['y'], and ['theta']. Result is stored into
        vehicle_state['lidarPointsWorld'] as an Nx2 numpy array.
        """
        x = vehicle_state['x']
        y = vehicle_state['y']
        theta = -vehicle_state['theta']

        r = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        t = np.array([x, y])

        sweep_vehicle_frame = vehicle_state['lidarPointsCartesian']
        sweep_world_frame = (sweep_vehicle_frame @ r) + t
        vehicle_state['lidarPointsWorld'] = sweep_world_frame

    def build_sweep(self, vehicle_state):
        """
        Takes the points contained in vehicle_state['lidarPointsWorld'] and appends it to the sweep currently being
        built. If we now have a full sweep, places it into self.full_sweep
        sweep.
        """
        for sphericalVehicleFramePoint, cartesianWorldFramePoint in zip(vehicle_state['lidarPointsFiltered'],
                                                                        vehicle_state['lidarPointsWorld']):
            azimuth = sphericalVehicleFramePoint[0]
            if self.prev_azimuth is not None and azimuth < self.prev_azimuth:
                self.full_sweep = np.array(self.partial_sweep)
                self.partial_sweep = []

            self.partial_sweep.append(cartesianWorldFramePoint)
            self.prev_azimuth = azimuth
