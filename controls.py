#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np


class Controls:
    def __init__(self):
        pass

    def run(self, pose, waypoint, vehicle_commands):
        curr_heading = np.deg2rad(pose['theta'] % 360) - np.pi
        vec_start_to_goal = np.array(waypoint) - np.array([pose['x'], pose['y']])
        desired_heading = np.arctan2(vec_start_to_goal[1], vec_start_to_goal[0])

        left_drive_motor_speed = 25
        right_drive_motor_speed = 25
        if abs(curr_heading - desired_heading) >= 0.1:
            # 1. If we're not facing the right way, turn in place
            left_drive_motor_speed = -left_drive_motor_speed

        vehicle_commands["leftDriveMotorSpeed"] = left_drive_motor_speed
        vehicle_commands["rightDriveMotorSpeed"] = right_drive_motor_speed
        vehicle_commands["intakeCenterMotorSpeed"] = 512
        vehicle_commands["intakeLeftMotorSpeed"] = 512
        vehicle_commands["intakeRightMotorSpeed"] = 512

        """
        Commands dict looks something like this:
        vehicle_commands = {
            "leftDriveMotorSpeed": 0,  # Left drive motor speed (-512 - 512)
            "rightDriveMotorSpeed": 0,  # Right drive motor speed (-512 - 512)
            "intakeCenterMotorSpeed": 0,  # Intake center motor speed (-512 - 512)
            "intakeLeftMotorSpeed": 0,  # Intake left motor speed (-512 - 512)
            "intakeRightMotorSpeed": 0,  # Intake right motor speed (-512 - 512)
            "tubeMotorSpeed": 0,  # Tube motor speed (-512 - 512)
            "timerStartStop": 0,  # Timer start/stop (0 or 1)
            "reset": 0,  # Reset (0 or 1)
            "outtake": 0  # Outtake (0 or 1)
        }
        """
