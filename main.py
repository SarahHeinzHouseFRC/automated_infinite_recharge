#!/usr/bin/python3

#
# Copyright (c) 2020 FRC Team 3260
#

import argparse
from comms import *
from config import Config
from perception import Perception
from planning import Planning
from controls import Controls
from visualize import Visualize

CONFIG_FILE = "config.yml"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--player", help="player 1-6", type=int, default=1)
    args = parser.parse_args()

    config = Config(CONFIG_FILE, args.player)

    comms_config = {
        'rx_ip': config.client_ip,
        'rx_port': config.client_port,
        'tx_ip': config.sim_ip,
        'tx_port': config.sim_port
    }
    print("Rx at {}:{}".format(comms_config["rx_ip"], comms_config["rx_port"]))
    print("Tx to {}:{}".format(comms_config["tx_ip"], comms_config["tx_port"]))

    # Launch comms in background thread
    comms = CommsThread(comms_config, False)
    comms.daemon = True
    comms.start()

    # Launch perception, motion planning, and controls in main thread
    perception = Perception(config)
    planning = Planning(config)
    controls = Controls()
    visualize = Visualize()

    try:
        while True:
            if len(comms.vehicle_state['lidarSweep']) > 0:
                world_state = perception.run(comms.vehicle_state)
                plan_state = planning.run(world_state)
                controls.run(plan_state, comms.vehicle_commands)

                comms.vehicle_commands['draw'] = visualize.run(world_state, plan_state)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
