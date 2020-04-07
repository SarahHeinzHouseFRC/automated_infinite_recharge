#
# Copyright (c) 2020 FRC Team 3260
#


class Controls:
    def __init__(self, sim_commands):
        self.sim_commands = sim_commands

    def run(self):
        try:
            while True:
                self.sim_commands.leftDriveMotorSpeed = 256
                self.sim_commands.rightDriveMotorSpeed = -256
            # while True:
                # self.sim_commands.leftDriveMotorSpeed += 1
                # if self.sim_commands.leftDriveMotorSpeed >= 512:
                #     self.sim_commands.leftDriveMotorSpeed = 0
        except KeyboardInterrupt:
            pass
