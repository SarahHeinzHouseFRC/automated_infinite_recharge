#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
import geometry as geom


class Planning:
    def __init__(self, config):
        self.field = config.outer_wall
        self.field_columns = config.field_columns
        self.field_trenches = config.field_trenches
        self.red_goal_region = config.red_goal_region
        self.blue_goal_region = config.blue_goal_region
        self.scoring_zone = self.blue_goal_region.center + np.array([0, 1])
        self.blue_player_station_pos = config.blue_player_station_pos - np.array([0, 0.5])

        self.prev_obstacles = None  # Used to remember balls that were nearby but are now in LIDAR deadzone
        self.prev_goal = None  # Used to prevent flip-flopping between two equidistant goals
        self.occupancy_grid_dilation_kernel_size = config.occupancy_grid_dilation_kernel_size
        self.ball_grid = geom.OccupancyGrid(config.occupancy_grid_width,
                                            config.occupancy_grid_height,
                                            config.occupancy_grid_cell_resolution,
                                            config.occupancy_grid_origin)
        self.temp_obstacle_grid = geom.OccupancyGrid(config.occupancy_grid_width,
                                                     config.occupancy_grid_height,
                                                     config.occupancy_grid_cell_resolution,
                                                     config.occupancy_grid_origin)
        self.obstacle_grid = geom.OccupancyGrid(config.occupancy_grid_width,
                                                config.occupancy_grid_height,
                                                config.occupancy_grid_cell_resolution,
                                                config.occupancy_grid_origin)

        self.deadzone_radius = config.lidar_deadzone_radius
        self.aggregated_balls = dict()
        self.aggregated_others = dict()

        self.ball_probability_decay_factor = config.ball_probability_decay_factor
        self.ball_probability_growth_factor = config.ball_probability_growth_factor
        self.ball_probability_threshold = config.ball_probability_threshold

        self.obstacle_probability_decay_factor = config.obstacle_probability_decay_factor
        self.obstacle_probability_growth_factor = config.obstacle_probability_growth_factor
        self.obstacle_probability_threshold = config.obstacle_probability_threshold

    def run(self, world_state):
        """
        Picks a goal to drive towards and then determines a collision-free trajectory for driving to that goal.
        :param world_state: Output of perception
        :return: Dict containing the robot's current pose and a goal state as the input of controls
        """
        # 1. Identify the goal
        self.behavior_planning(world_state)

        # 2. Move towards it if there is a nearest ball (using A*)
        self.motion_planning(world_state)

        plan = {
            'pose': world_state['pose'],
            'trajectory': world_state['trajectory'],
            'grid': world_state['grid'],
            'goal': world_state['goal'],
            'direction': world_state['direction'],
            'tube_mode': world_state['tube_mode'],
            'flail': world_state['flail'],
        }
        return plan

    def behavior_planning(self, world_state):
        """
        Identifies a goal state and places it into world_state['goal']. Also identifies whether to run the intake or
        outtake and places it into world_state['tube_mode'] as one of 'INTAKE', 'OUTTAKE', 'NONE'. Also identifies which
        direction to drive in, one of '1', '-1', or '0'
        """

        start = world_state['pose'][0]  # Our current (x,y)
        goal = None
        direction = None
        tube_mode = None

        if geom.dist(start, self.scoring_zone) <= 0.15 and world_state['numIngestedBalls'] > 0:
            # If we're in the scoring zone with some balls then run the outtake
            tube_mode = 'OUTTAKE'
            direction = 0
            goal = self.scoring_zone

        elif world_state['numIngestedBalls'] > 4:
            # If we have >4 balls then drive backwards towards the goal
            tube_mode = 'INTAKE'
            direction = -1
            goal = self.scoring_zone

        else:
            # The rest of the time, just run the intake and go towards the closest unobstructed ball
            tube_mode = 'INTAKE'
            direction = 1

            # Recover any balls that are now within the deadzone
            if self.prev_obstacles is not None:
                for ball in self.prev_obstacles:
                    if 0.5 < geom.dist(start, ball) <= self.deadzone_radius and ball not in world_state['balls']:
                        world_state['balls'].append(ball)
            self.prev_obstacles = world_state['balls']

            # Decay the probability of all balls
            self.ball_grid.occupancy -= self.ball_probability_decay_factor
            self.ball_grid.occupancy = np.clip(self.ball_grid.occupancy, a_min=0, a_max=1)

            # Increase the probability of all cells currently containing balls
            for ball_pos in world_state['balls']:
                cell_indices = self.ball_grid.get_cell(ball_pos).indices
                self.ball_grid.occupancy[cell_indices] += self.ball_probability_growth_factor
            self.ball_grid.occupancy = np.clip(self.ball_grid.occupancy, a_min=0, a_max=1)

            # Get final ball positions
            world_state['balls'] = []
            for i in range(self.ball_grid.occupancy.shape[0]):
                for j in range(self.ball_grid.occupancy.shape[1]):
                    cell_position = self.ball_grid.grid[i][j].position
                    p = self.ball_grid.occupancy[i][j]
                    if p >= self.ball_probability_threshold:
                        world_state['balls'].append(cell_position)

            # Find the closest ball
            min_dist = np.inf
            for ball_pos in world_state['balls']:
                curr_dist = geom.dist(ball_pos, start)
                if curr_dist < min_dist and self.obstacle_grid.get_occupancy(ball_pos) < self.obstacle_probability_threshold:
                    min_dist = curr_dist
                    goal = ball_pos

        # If we're currently inside an obstacle, flail!
        do_flail = self.obstacle_grid.get_occupancy(start) >= self.obstacle_probability_threshold

        # Last resort!
        if goal is None:
            # If we can't reach any balls then go towards the blue player station
            tube_mode = 'INTAKE'
            direction = 1
            goal = self.blue_player_station_pos

        world_state['flail'] = do_flail
        world_state['goal'] = goal
        world_state['direction'] = direction
        world_state['tube_mode'] = tube_mode

    def motion_planning(self, world_state):
        """
        Calculates a trajectory for driving from the current position to the goal contained in world_state['goal']. The
        result is placed into world_state['trajectory']
        :param world_state: Outputs of perception and behavior planning
        """
        if world_state['flail']:
            world_state['trajectory'] = None
            world_state['grid'] = self.obstacle_grid
            return

        # Add all obstacles to a fresh obstacle grid and inflate them
        self.temp_obstacle_grid.occupancy.fill(0)
        for static_obstacle in self.field_columns:
            self.temp_obstacle_grid.insert_convex_polygon(static_obstacle, 1.0)
        for static_obstacle in self.field_trenches:
            self.temp_obstacle_grid.insert_rectangular_obstacle(static_obstacle.bounding_box, 1.0)
        for dynamic_obstacle in world_state['obstacles']:
            self.temp_obstacle_grid.insert_rectangular_obstacle(dynamic_obstacle, self.obstacle_probability_growth_factor)
        self.temp_obstacle_grid.inflate_obstacles(kernel_size=self.occupancy_grid_dilation_kernel_size)

        # Decay the probability of all cells in the master obstacle grid
        self.obstacle_grid.clear_node_parents()
        self.obstacle_grid.occupancy -= self.obstacle_probability_decay_factor
        self.obstacle_grid.occupancy = np.clip(self.obstacle_grid.occupancy, a_min=0, a_max=1)

        # Update the master obstacle grid with our temp one
        self.obstacle_grid.occupancy += self.temp_obstacle_grid.occupancy
        self.obstacle_grid.occupancy = np.clip(self.obstacle_grid.occupancy, a_min=0, a_max=1)

        # Call A* to generate a path to goal
        trajectory = None
        if world_state['goal'] is not None:
            start = world_state['pose'][0]
            goal = world_state['goal']
            trajectory = geom.a_star(self.obstacle_grid, self.obstacle_probability_threshold, start, goal)

        if trajectory:
            trajectory = geom.smooth_trajectory(trajectory, self.obstacle_grid, self.obstacle_probability_threshold)

        world_state['trajectory'] = trajectory
        world_state['grid'] = self.obstacle_grid
