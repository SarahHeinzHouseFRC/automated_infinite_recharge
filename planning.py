#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
import geometry as geom


class Planning:
    def __init__(self, config):
        self.field = config.outer_wall
        self.field_elements = config.field_elements
        self.red_goal_region = config.red_goal_region
        self.blue_goal_region = config.blue_goal_region
        self.scoring_zone = self.blue_goal_region.center + np.array([0, 1])

        self.prev_obstacles = None  # Used to remember balls that were nearby but are now in LIDAR deadzone
        self.prev_goal = None  # Used to prevent flip-flopping between two equidistant goals
        self.occupancy_grid_dilation_kernel_size = config.occupancy_grid_dilation_kernel_size
        self.occupancy_grid = geom.OccupancyGrid(config.occupancy_grid_width,
                                                 config.occupancy_grid_height,
                                                 config.occupancy_grid_cell_resolution,
                                                 config.occupancy_grid_origin)

        self.deadzone_radius = config.lidar_deadzone_radius

    def run(self, world_state):
        # 1. Identify the goal
        self.behavior_planning(world_state)

        # 2. Move towards it if there is a nearest ball (using A*)
        self.motion_planning(world_state)

        plan_state = {
            'pose': world_state['pose'],
            'trajectory': world_state['trajectory'],
            'grid': world_state['grid'],
            'goal': world_state['goal'],
            'direction': world_state['direction'],
            'tube_mode': world_state['tube_mode']
        }
        return plan_state

    def behavior_planning(self, world_state):
        """
        Identifies a goal state and places it into world_state['goal'].

        Also identifies whether to run the intake or outtake and places it into world_state['tube_mode'] as
        one of 'INTAKE', 'OUTTAKE', 'NONE'.

        Also identifies which direction to drive in, one of '1', '-1', or '0'
        """
        start = world_state['pose'][0]  # Our current (x,y)
        goal = None
        direction = None
        tube_mode = None

        if geom.dist(start, self.scoring_zone) <= 0.15 and world_state['ingestedBalls'] > 0:
            # If we're in the scoring zone with some balls then run the outtake
            tube_mode = 'OUTTAKE'
            direction = 0
            goal = self.scoring_zone

        elif world_state['ingestedBalls'] > 4 or ():
            # If we have >4 balls then drive backwards towards the goal
            tube_mode = 'INTAKE'
            direction = -1
            goal = self.scoring_zone

        else:
            # The rest of the time, just run the intake and go towards the closest ball
            tube_mode = 'INTAKE'
            direction = 1
            # 1. Add some object persistence so balls inside the LIDAR deadzone don't keep going out of view
            if self.prev_obstacles is not None:
                # Run through and recover any balls within the deadzone and place them into world_state
                for ball in self.prev_obstacles:
                    if 0.5 < geom.dist(start, ball[0]) <= self.deadzone_radius:
                        world_state['obstacles']['balls'].append(ball)
            self.prev_obstacles = world_state['obstacles']['balls']

            # 2. Find the closest ball
            min_dist = np.inf
            for ball_pos, ball_radius in world_state['obstacles']['balls']:
                curr_dist = geom.dist(ball_pos, start)
                if curr_dist < min_dist and not self.occupancy_grid.get_occupancy(ball_pos):
                    min_dist = curr_dist
                    goal = ball_pos

        world_state['goal'] = goal
        world_state['direction'] = direction
        world_state['tube_mode'] = tube_mode

    def motion_planning(self, world_state):
        """
        Identifies a motion plan for achieving the goal state contained in world_state['goal'] and places a
        trajectory waypoint into world_state['waypoint'].
        """
        # clear the positions previously marked as obstacles because they may have changed
        self.occupancy_grid.clear()

        # Insert static obstacles
        for static_obstacle in self.field_elements:
            self.occupancy_grid.insert_convex_polygon(static_obstacle)

        # Insert dynamic obstacles
        dynamic_obstacles = world_state['obstacles']['others']
        for dynamic_obstacle in dynamic_obstacles:
            self.occupancy_grid.insert_rectangular_obstacle(dynamic_obstacle)

        self.occupancy_grid.dilate(kernel_size=self.occupancy_grid_dilation_kernel_size)

        # Call A* to generate a path to goal
        trajectory = None
        if world_state['goal'] is not None:
            start = world_state['pose'][0]
            goal = world_state['goal']
            trajectory = geom.a_star(self.occupancy_grid, start, goal)

        if trajectory:
            trajectory = geom.smooth_trajectory(trajectory)

        world_state['trajectory'] = trajectory
        world_state['grid'] = self.occupancy_grid
