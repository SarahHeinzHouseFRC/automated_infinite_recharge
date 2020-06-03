#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import algorithm as alg

IN_TO_M = 0.0254


class Planning:
    def __init__(self):
        plt.ion()
        self.prev_obstacles = None
        self.grid = alg.Grid(width=IN_TO_M*2*161.81, height=IN_TO_M*2*314.96, cell_resolution=0.1, origin=(0,0))

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

        self.field_elements = [self.outer_wall, self.right_column, self.top_column, self.left_column, self.bottom_column,
                self.right_trench_right_wall, self.right_trench_left_wall,
                self.left_trench_right_wall, self.left_trench_left_wall]

    def run(self, world_state):
        # 1. Identify the goal
        self.behavior_planning(world_state)

        # 2. Move towards it if there is a nearest ball (using A*)
        self.motion_planning(world_state)

        # self.visualize(world_state)

        plan_state = {
            'pose': world_state['pose'],
            'trajectory': world_state['trajectory']
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
        static_obstacles = [alg.bounding_box(static_obstacle) for static_obstacle in self.field_elements]
        self.grid.insert_obstacles(static_obstacles)

        # Insert dynamic obstacles
        dynamic_obstacles = world_state['obstacles']['others']
        self.grid.insert_obstacles(dynamic_obstacles)

        # call a* to generate a path to goal
        if world_state['goal'] is not None:
            start_node = self.grid.get_cell(world_state['pose'][0])
            goal_node = self.grid.get_cell(world_state['goal'])
            node_path = alg.a_star(self.grid, start_node, goal_node)
            trajectory = [node.position for node in node_path]
            world_state['trajectory'] = trajectory
        else:
            world_state['trajectory'] = None

    def visualize(self, world_state):
        plt.clf()
        fig = plt.gcf()
        ax = fig.gca()

        # Plot field
        plt.fill(self.outer_wall[:, 0], self.outer_wall[:, 1], facecolor=(0,0,0,0), edgecolor='gray', linewidth=1)

        # Plot vehicle position
        vehicle_x, vehicle_y = world_state['pose'][0]
        vehicle_position = np.array((vehicle_x, vehicle_y))
        plt.plot(vehicle_position[0], vehicle_position[1], color=(1.0, 0.37, 0.22, 1.0), marker='x', linestyle='')
        plt.text(vehicle_x, vehicle_y + 0.05, 'Self', color='r', fontsize=10)

        # Plot obstacles
        for ball in world_state['obstacles']['balls']:
            ball_x, ball_y = ball[0]
            ball_radius = ball[1]
            box_x = ball_x - ball_radius - 0.05
            box_y = ball_y - ball_radius - 0.05
            box_width = 2 * ball_radius + 0.1
            box_height = 2 * ball_radius + 0.1
            bbox = patches.Rectangle((box_x, box_y), box_width, box_height, linewidth=1, edgecolor='r', facecolor='none')
            ax.add_patch(bbox)
            plt.text(box_x, box_y + box_height + 0.05, 'Ball', color='r', fontsize=10)
        for perception_object in world_state['obstacles']['others']:
            min_x, max_x, min_y, max_y = perception_object
            width = max_x - min_x
            height = max_y - min_y
            bbox = patches.Rectangle((min_x, min_y), width, height, linewidth=1, edgecolor='b', facecolor='none')
            ax.add_patch(bbox)
            plt.text(min_x, min_y + height + 0.05, 'Other', color='b', fontsize=10)

        # Plot goal
        if world_state['goal'] is not None:
            goal_x, goal_y = world_state['goal']
            plt.plot(goal_x, goal_y, color='g', marker='.', linestyle='')
            plt.text(goal_x, goal_y + 0.05, 'Goal', color='g', fontsize=10)

            x = [vehicle_x, goal_x]
            y = [vehicle_y, goal_y]
            plt.plot(x, y, color='g')

        plt.title('Team SHARP FRC 2020 Planning Stack')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.axis('equal')
        plt.draw()
        plt.pause(0.1)
