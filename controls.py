#
# Copyright (c) 2020 FRC Team 3260
#

import numpy as np


class Controls:
    def __init__(self):
        pass

    def run(self, plan_state, vehicle_commands):
        if plan_state['trajectory'] is None:
            return

        pose = plan_state['pose']
        start = np.array(pose[0])
        goal = np.array(plan_state['trajectory'])

        curr_heading = pose[1] % (2*np.pi)
        vec_start_to_goal = goal - start
        desired_heading = np.arctan2(vec_start_to_goal[1], vec_start_to_goal[0]) % (2*np.pi)

        # If we're not facing the right way, turn in place. Else move straight
        left_drive_motor_speed = 25
        right_drive_motor_speed = 25

        margin = 0.25
        scale = 20

        if abs(desired_heading - curr_heading) >= margin:
            left_drive_motor_speed = int((curr_heading - desired_heading) * scale)
            right_drive_motor_speed = int((desired_heading - curr_heading) * scale)

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
            "draw": []  # List of shapes to draw
        }
        """
