#
# Copyright (c) 2020 FRC Team 3260
#

from comms import *
from threading import Thread
import time


class CommsThread(Thread):
    def __init__(self, comms_config, sim_commands, verbose):
        super(CommsThread, self).__init__()
        self.verbose = verbose
        self.comms = Comms(comms_config)
        self.sim_commands = sim_commands

    def run(self):
        while True:
            # Send sim commands
            tx_msg = self.sim_commands.toJson()
            self.comms.tx(tx_msg)
            if self.verbose:
                print("Sent: ", tx_msg)

            # Receive sensor state
            rx_msg = self.comms.rx()
            if self.verbose and rx_msg is not None:
                print("Received: ", rx_msg)

            # Sleep for safety
            time.sleep(0.01)

    def join(self, **kwargs):
        Thread.join(self)
