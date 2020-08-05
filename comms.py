#
# Copyright (c) 2020 FRC Team 3260
#

from threading import Thread
import time
import socket
import json


class CommsThread(Thread):
    def __init__(self, comms_config, verbose, commands_mutex):
        super(CommsThread, self).__init__()
        self.commands_mutex = commands_mutex
        self.verbose = verbose
        self.comms = Comms(comms_config)
        # Since we're initalizing, we don't need to hold the commands_mutex
        # yet, because no one else has access to this data structure yet
        self.vehicle_commands = {
            'leftDriveMotorSpeed': 0,  # Left drive motor speed (-512 - 512)
            'rightDriveMotorSpeed': 0,  # Right drive motor speed (-512 - 512)
            'intakeCenterMotorSpeed': 0,  # Intake center motor speed (-512 - 512)
            'intakeLeftMotorSpeed': 0,  # Intake left motor speed (-512 - 512)
            'intakeRightMotorSpeed': 0,  # Intake right motor speed (-512 - 512)
            'tubeMotorSpeed': 0,  # Tube motor speed (-512 - 512)
            'timerStartStop': 0,  # Timer start/stop (0 or 1)
            'reset': 0,  # Reset (0 or 1)
            'outtake': 0,  # Outtake (0 or 1)
            'draw': []  # List of shapes to be drawn
        }
        self.vehicle_states = []

    def run(self):
        while True:
            # Send sim commands
            with self.commands_mutex:
                # hold the lock while we're working on this,
                # so the planning code doesn't modify it while
                # we're in the middle of transmitting it
                tx_msg = json.dumps(self.vehicle_commands, separators=(',', ':'))
            self.comms.tx(tx_msg)
            if self.verbose:
                print('Sent: {tx_msg}')

            # Receive sensor state
            rx_msg = self.comms.rx()
            if rx_msg is not None:
                vehicle_state = json.loads(rx_msg)
                self.vehicle_states.append(vehicle_state)
                if self.verbose:
                    print('Received: {rx_msg}')
                # print('Received: ', vehicle_state['leftDriveEncoder'])

            # Sleep for safety
            # time.sleep(0.001)

    def join(self, **kwargs):
        Thread.join(self)

    def get_vehicle_states(self):
        """
        Returns all the vehicle states received since the last call of this method
        """
        ret = self.vehicle_states.copy()
        self.vehicle_states = []
        return ret


class Comms:
    def __init__(self, comms_config):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self.socket.bind((comms_config['rx_ip'], comms_config['rx_port']))
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.settimeout(0.1)
        self.tx_ip = comms_config['tx_ip']
        self.tx_port = comms_config['tx_port']

    def tx(self, msg):
        self.socket.sendto(bytes(msg, encoding='utf8'), (self.tx_ip, self.tx_port))

    def rx(self):
        buffer_size = 65536
        try:
            rx_msg, addr = self.socket.recvfrom(buffer_size)
            return rx_msg
        except socket.timeout:
            return None
