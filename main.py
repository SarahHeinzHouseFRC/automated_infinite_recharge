#!/usr/bin/python3

#
# Copyright (c) 2020 FRC Team 3260
#

from comms import *
from perception import Perception
from planning import Planning
from controls import Controls


def main():
    comms_config = {
        "rx_ip": "127.0.0.1",
        "rx_port": 6000,
        "tx_ip": "127.0.0.1",
        "tx_port": 8000
    }

    # Launch comms in background thread
    comms = CommsThread(comms_config, False)
    comms.daemon = True
    comms.start()

    # Launch perception, motion planning, and controls in main thread
    perception = Perception()
    planning = Planning()
    controls = Controls()

    try:
        while True:
            if len(comms.vehicle_state["lidarSweep"]) > 0:
                world_state = perception.run(comms.vehicle_state)
                plan_state = planning.run(world_state)
                controls.run(plan_state, comms.vehicle_commands)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
