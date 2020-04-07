#
# Copyright (c) 2020 FRC Team 3260
#

import socket
import json


class Comms:
    def __init__(self, comms_config):
        self.rx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self.rx_socket.bind((comms_config['rx_ip'], comms_config['rx_port']))
        self.rx_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx_socket.settimeout(0.1)
        self.tx_ip = comms_config['tx_ip']
        self.tx_port = comms_config['tx_port']
        self.tx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP

    def tx(self, msg):
        self.tx_socket.sendto(bytes(msg, encoding='utf8'), (self.tx_ip, self.tx_port))

    def rx(self):
        buffer_size = 1024
        try:
            rx_msg, addr = self.rx_socket.recvfrom(buffer_size)
            return rx_msg
        except socket.timeout:
            return None


class SimCommands:
    """
    Send commands to the sim in the following form:
    {
        "leftDriveMotorSpeed": 0,
        "rightDriveMotorSpeed": 0,
        "intakeCenterMotorSpeed": 0,
        "intakeLeftMotorSpeed": 0,
        "intakeRightMotorSpeed": 0,
        "tubeMotorSpeed": 0,
        "timerStartStop": 0,
        "reset": 0,
        "outtake": 0
    }
    """
    def __init__(self):
        self.leftDriveMotorSpeed = 0  # Left drive motor speed (-512 - 512)
        self.rightDriveMotorSpeed = 0  # Right drive motor speed (-512 - 512)
        self.intakeCenterMotorSpeed = 0  # Intake center motor speed (-512 - 512)
        self.intakeLeftMotorSpeed = 0  # Intake left motor speed (-512 - 512)
        self.intakeRightMotorSpeed = 0  # Intake right motor speed (-512 - 512)
        self.tubeMotorSpeed = 0  # Tube motor speed (-512 - 512)
        self.timerStartStop = 0  # Timer start/stop (0 or 1)
        self.reset = 0  # Reset (0 or 1)
        self.outtake = 0  # Outtake (0 or 1)

    def toJson(self):
        result = {
            "leftDriveMotorSpeed": self.leftDriveMotorSpeed,
            "rightDriveMotorSpeed": self.rightDriveMotorSpeed,
            "intakeCenterMotorSpeed": self.intakeCenterMotorSpeed,
            "intakeLeftMotorSpeed": self.intakeLeftMotorSpeed,
            "intakeRightMotorSpeed": self.intakeRightMotorSpeed,
            "tubeMotorSpeed": self.tubeMotorSpeed,
            "timerStartStop": self.timerStartStop,
            "reset": self.reset,
            "outtake":self.outtake
        }
        return json.dumps(result)
