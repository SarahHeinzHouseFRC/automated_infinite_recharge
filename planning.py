#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
import geometry as geom

IN_TO_M = 0.0254


class Planning:
    def __init__(self, config):
        self.field = config.outer_wall
        self.field_elements = config.field_elements
        self.red_goal_region = config.red_goal_region
        self.blue_goal_region = config.blue_goal_region
        self.static_obstacles = [config.right_trench_right_wall,
                                 config.right_trench_left_wall,
                                 config.left_trench_right_wall,
                                 config.left_trench_left_wall,
                                 config.right_column,
                                 config.left_column,
                                 config.top_column,
                                 config.bottom_column]

        self.prev_obstacles = None
        self.grid = geom.Grid(width=10, height=16, cell_resolution=0.1, origin=(0,0))

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

        Also identify what action we want to take, as world_state['tube_mode'] which
        is one of 'INTAKE', 'OUTTAKE', 'NONE'. Also identify which direction to drive in,
        one of '1', '-1', or '0'
        """
        start = world_state['pose'][0]  # Our current (x,y)
        scoring_zone = (self.blue_goal_region.center[0], self.blue_goal_region.center[1] + 1)

        '''
        ### lazy f-strings for programmers and debug output (new in python3 (3.8?)):
        In [3]: x = {1:2, 3:4}
        In [5]: y = "nice"
        In [6]: print(f"{x[1]=}, {y=}")
        x[1]=2, y='nice'
        '''
        if world_state['ingestedBalls'] > 4 or (geom.dist(start, scoring_zone) <= 0.15 and world_state['ingestedBalls'] > 0):

            # If we're close to pregoal then run the tube
            if geom.dist(start, scoring_zone) <= 0.15:
                tube_mode = 'OUTTAKE'
                direction = 0
                goal = scoring_zone
            # Else go towards pregoal
            else:
                tube_mode = 'INTAKE'
                direction = -1
                goal = scoring_zone
        else:
            tube_mode = 'INTAKE'
            direction = 1
            # 1. Add some object persistence so balls inside the LIDAR deadzone don't keep going out of view
            deadzone_radius = 0.85
            if self.prev_obstacles is not None:
                # Run through and recover any balls within the deadzone and place them into world_state
                for ball in self.prev_obstacles:
                    if 0.5 < geom.dist(start, ball[0]) < deadzone_radius:
                        world_state['obstacles']['balls'].append(ball)
            self.prev_obstacles = world_state['obstacles']['balls']

            # 2. Find the closest ball
            min_dist = np.inf
            goal = None
            for ball in world_state['obstacles']['balls']:
                curr_dist = geom.dist(ball[0], start)
                if curr_dist < min_dist:
                    min_dist = curr_dist
                    goal = ball[0]

        world_state['goal'] = goal
        world_state['direction'] = direction
        world_state['tube_mode'] = tube_mode

    def motion_planning(self, world_state):
        """
        Identifies a motion plan for achieving the goal state contained in world_state['goal'] and places a
        trajectory waypoint into world_state['waypoint'].
        """
        # clear the positions previously marked as obstacles because they may have changed
        self.grid.clear()

        # Insert static obstacles
        for static_obstacle in self.static_obstacles:
            self.grid.insert_convex_polygon(static_obstacle)

        # Insert dynamic obstacles
        dynamic_obstacles = world_state['obstacles']['others']
        for obst in dynamic_obstacles:
            self.grid.insert_rectangular_obstacle(obst)
        self.grid.dilate(kernel_size=7)

        # Call A* to generate a path to goal
        trajectory = None
        if world_state['goal'] is not None:
            start = world_state['pose'][0]
            goal = world_state['goal']
            start_node = self.grid.get_cell(start)
            goal_node = self.grid.get_cell(goal)
            node_path = geom.a_star(self.grid, start_node, goal_node)
            if node_path:
                trajectory = [node.position for node in node_path]
                trajectory[0] = start
                trajectory[-1] = goal

        world_state['trajectory'] = trajectory
        world_state['grid'] = self.grid
