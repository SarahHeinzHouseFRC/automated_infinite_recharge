#
# Copyright (c) 2020 FRC Team 3260
#

import yaml
import numpy as np
from geometry import Polygon

IN_TO_M = 0.0254


class Config:
    def __init__(self, fname, player):
        """
        Reads the config file at the given path and parses its fields into usable types
        :param fname: Filename of the config file
        :param player: Player index 1-6
        """
        with open(fname) as f:
            config = yaml.safe_load(f)

            # Comms
            self.client_ip = config['client']['ip']
            self.client_port = config['client']['port'] + 10 * (player-1)
            self.sim_ip = config['sim']['ip']
            self.sim_port = config['sim']['port'] + 10 * (player-1)

            # Field shape
            self.outer_wall = Polygon(IN_TO_M * np.array(config['sim']['field']['exteriorWall']))
            self.right_trench_right_wall = Polygon(IN_TO_M * np.array(config['sim']['field']['rightTrenchRightWall']))
            self.right_trench_left_wall = Polygon(IN_TO_M * np.array(config['sim']['field']['rightTrenchLeftWall']))
            self.left_trench_right_wall = Polygon(IN_TO_M * np.array(config['sim']['field']['leftTrenchRightWall']))
            self.left_trench_left_wall = Polygon(IN_TO_M * np.array(config['sim']['field']['leftTrenchLeftWall']))
            self.right_column = Polygon(IN_TO_M * np.array(config['sim']['field']['rightColumn']))
            self.left_column = Polygon(IN_TO_M * np.array(config['sim']['field']['leftColumn']))
            self.top_column = Polygon(IN_TO_M * np.array(config['sim']['field']['topColumn']))
            self.bottom_column = Polygon(IN_TO_M * np.array(config['sim']['field']['bottomColumn']))

            self.field_columns = [self.right_column,
                                  self.left_column,
                                  self.top_column,
                                  self.bottom_column]
            self.field_trenches = [self.right_trench_right_wall,
                                   self.right_trench_left_wall,
                                   self.left_trench_right_wall,
                                   self.left_trench_left_wall]
            self.field_elements = [self.right_trench_right_wall,
                                   self.right_trench_left_wall,
                                   self.left_trench_right_wall,
                                   self.left_trench_left_wall,
                                   self.right_column,
                                   self.left_column,
                                   self.top_column,
                                   self.bottom_column]

            self.red_goal_region = Polygon(IN_TO_M * np.array(config['sim']['field']['redGoalRegion']))
            self.blue_goal_region = Polygon(IN_TO_M * np.array(config['sim']['field']['blueGoalRegion']))

            self.blue_player_station_pos = IN_TO_M * np.array([-61.00, 314.96])
            self.red_player_station_pos = IN_TO_M * np.array([61.00, -314.96])

            # Occupancy grid
            self.occupancy_grid_width = 10
            self.occupancy_grid_height = 16
            self.occupancy_grid_cell_resolution = 0.1
            self.occupancy_grid_origin = (0, 0)
            self.occupancy_grid_dilation_kernel_size = 13

            self.ball_probability_decay_factor = 0.01
            self.ball_probability_growth_factor = 0.2
            self.ball_probability_threshold = 0.8

            self.obstacle_probability_decay_factor = 0.05
            self.obstacle_probability_growth_factor = 0.1
            self.obstacle_probability_threshold = 0.8

            # LIDAR
            self.lidar_deadzone_radius = 0.85

            # Controls
            self.drive_kp = 40
            self.drive_ki = 1
            self.drive_kd = 1
            self.heading_error_threshold = 0.1
            self.max_forward_speed = 100
            self.max_intake_speed = 512
            self.max_outtake_speed = 512

            # Game pieces
            self.ball_radius = IN_TO_M * config['sim']['gamePiece']['radius']
