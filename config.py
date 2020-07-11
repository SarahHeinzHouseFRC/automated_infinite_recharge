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

            self.occupancy_grid_width = 10
            self.occupancy_grid_height = 16
            self.occupancy_grid_cell_resolution = 0.1
            self.occupancy_grid_origin = (0, 0)
            self.occupancy_grid_dilation_kernel_size = 7

            # Game pieces
            self.ball_radius = IN_TO_M * config['sim']['gamePiece']['radius']
