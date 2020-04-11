#
# Copyright (c) 2020 FRC Team 3260
#


class Controls:
    def __init__(self):
        pass

    def run(self, waypoint, vehicle_commands):
        vehicle_commands["leftDriveMotorSpeed"] = 10
        vehicle_commands["rightDriveMotorSpeed"] = 5
