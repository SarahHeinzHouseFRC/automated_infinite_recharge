#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
import algorithm as alg
from geometry import Polygon

IN_TO_M = 0.0254


class Planning:
    def __init__(self, config):
        self.field = Polygon(config.outer_wall)
        self.field_elements = [Polygon(_) for _ in config.field_elements]

        self.prev_obstacles = None
        self.grid = alg.Grid(width=10, height=16, cell_resolution=0.1, origin=(0,0))

    def run(self, world_state):
        # 1. Identify the goal
        self.behavior_planning(world_state)

        # 2. Move towards it if there is a nearest ball (using A*)
        self.motion_planning(world_state)

        plan_state = {
            'pose': world_state['pose'],
            'trajectory': world_state['trajectory'],
            'grid': world_state['grid']
        }
        return plan_state

    def behavior_planning(self, world_state):
        """
        Identifies a goal state and places it into world_state['goal'].
        """
        start = world_state['pose'][0]  # Our current (x,y)

        # 1. Add some object persistence so balls inside the LIDAR deadzone don't keep going out of view
        deadzone_radius = 0.85
        if self.prev_obstacles is not None:
            # Run through and recover any balls within the deadzone and place them into world_state
            for ball in self.prev_obstacles:
                if 0.5 < alg.dist(start, ball[0]) < deadzone_radius:
                    world_state['obstacles']['balls'].append(ball)
        self.prev_obstacles = world_state['obstacles']['balls']

        # 2. Find the closest ball
        min_dist = np.inf
        goal = None
        for ball in world_state['obstacles']['balls']:
            curr_dist = alg.dist(ball[0], start)
            if curr_dist < min_dist:
                min_dist = curr_dist
                goal = ball[0]
        world_state['goal'] = goal

    def motion_planning(self, world_state):
        """
        Identifies a motion plan for achieving the goal state contained in world_state['goal'] and places a
        trajectory waypoint into world_state['waypoint'].
        """
        # clear the positions previously marked as obstacles because they may have changed
        self.grid.clear()

        # Insert static obstacles
        # static_obstacles = [alg.bounding_box(static_obstacle) for static_obstacle in self.field_elements]
        # self.grid.insert_obstacles(static_obstacles)

        # Insert dynamic obstacles
        dynamic_obstacles = world_state['obstacles']['others']
        self.grid.insert_obstacles(dynamic_obstacles)

        # Call A* to generate a path to goal
        if world_state['goal'] is not None:
            start = world_state['pose'][0]
            goal = world_state['goal']
            start_node = self.grid.get_cell(start)
            goal_node = self.grid.get_cell(goal)
            node_path = alg.a_star(self.grid, start_node, goal_node)
            trajectory = [node.position for node in node_path]
            trajectory[0] = start
            trajectory[-1] = goal
            world_state['trajectory'] = trajectory
        else:
            world_state['trajectory'] = None

        world_state['grid'] = self.grid
