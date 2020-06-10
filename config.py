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

            self.client_ip = config['client']['ip']
            self.client_port = config['client']['port'] + 10 * (player-1)
            self.sim_ip = config['sim']['ip']
            self.sim_port = config['sim']['port'] + 10 * (player-1)
            self.outer_wall = IN_TO_M * np.array(config['sim']['field']['exteriorWall'])
            self.field_elements = [IN_TO_M * np.array(config['sim']['field']['rightTrenchRightWall']),
                                   IN_TO_M * np.array(config['sim']['field']['rightTrenchLeftWall']),
                                   IN_TO_M * np.array(config['sim']['field']['leftTrenchRightWall']),
                                   IN_TO_M * np.array(config['sim']['field']['leftTrenchLeftWall']),
                                   IN_TO_M * np.array(config['sim']['field']['rightColumn']),
                                   IN_TO_M * np.array(config['sim']['field']['leftColumn']),
                                   IN_TO_M * np.array(config['sim']['field']['topColumn']),
                                   IN_TO_M * np.array(config['sim']['field']['bottomColumn'])]

            self.red_goal_region = IN_TO_M * np.array(config['sim']['field']['redGoalRegion'])
            self.blue_goal_region = IN_TO_M * np.array(config['sim']['field']['blueGoalRegion'])
