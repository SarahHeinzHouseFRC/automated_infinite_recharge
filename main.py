#!/usr/bin/python3

#
# Copyright (c) 2020 FRC Team 3260
#

from comms import SimCommands
from comms_thread import CommsThread
from controls import Controls


def main():
    comms_config = {
        "rx_ip": "127.0.0.1",
        "rx_port": 6000,
        "tx_ip": "127.0.0.1",
        "tx_port": 8000
    }

    sim_commands = SimCommands()

    # Launch comms in background thread
    comms = CommsThread(comms_config, sim_commands, True)
    comms.daemon = True
    comms.start()

    # Launch controls in main thread
    controls = Controls(sim_commands)
    controls.run()


if __name__ == '__main__':
    main()
