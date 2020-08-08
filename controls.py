#
# Copyright (c) 2020 FRC Team 3260
#

import time
import geometry as geom
import numpy as np
import math


class PID:
    """
    PID object for iteratively getting from some current value to a desired value.
    """
    def __init__(self, kp, ki, kd):
        """
        Constructor
        :param kp: Proportional value as a float
        :param ki: Integral value as a float
        :param kd: Derivative value as a float
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_over_time_len = 10
        self.error_over_time = np.zeros(shape=(self.error_over_time_len,))
        self.prev_value = None
        self.prev_timestamp = None

    def run(self, curr_value, desired_value, curr_timestamp):
        """
        Calculates a signal to move curr_value towards desired_value. E.g. if curr_value is 10 and desired_value is 15,
        a positive signal will be returned. When curr_value is 20 and desired_value is 15, a negative signal will be
        returned. And if curr_value == desired_value, a signal of 0 will be returned. The current time is compared to
        the timestamp from the previous call of run() and used to calculate the derivative gain for this timestep.
        :param curr_value: Current position as a float
        :param desired_value: Desired position as a float
        :param curr_timestamp: Time run was called as a float
        :return Signal on how much to shift curr_value to move towards desired_value
        """
        # 1. Calculate the error
        error = desired_value - curr_value

        # 2. Calculate the proportional gain
        p_gain = self.kp * error

        # 3. Calculate the integral gain
        total_error_over_time = np.sum(self.error_over_time)
        i_gain = self.ki * total_error_over_time

        # 4. Calculate the derivative gain
        if self.prev_value is not None and self.prev_timestamp is not None:
            elapsed_time = curr_timestamp - self.prev_timestamp
            d_gain = (self.kd * (curr_value - self.prev_value))/elapsed_time
        else:
            d_gain = 0

        # 5. Sum them all up and return final value
        total_gain = p_gain + i_gain + d_gain

        # 6. Add this error into our list of errors over time and save curr_value into prev_value
        self.error_over_time = np.roll(self.error_over_time, 1)
        self.error_over_time[0] = error
        self.prev_value = curr_value
        self.prev_timestamp = curr_timestamp

        return total_gain


class Controls:
    """
    This class generates motor values from the output of planning, including for the two drive motors, intakes, and
    outtake.
    """
    def __init__(self, config):
        """
        Constructor
        :param config: Contains various constants from the robot
        """
        self.max_forward_speed = config.max_forward_speed
        self.max_intake_speed = config.max_intake_speed
        self.max_outtake_speed = config.max_outtake_speed
        self.heading_error_threshold = config.heading_error_threshold

        kp = config.drive_kp
        ki = config.drive_ki
        kd = config.drive_kd

        self.straight_pid = PID(500, 1, 1)
        self.left_drive_pid = PID(kp, ki, kd)
        self.right_drive_pid = PID(kp, ki, kd)

    def run(self, plan_state):
        """
        Calculates controls given the current plan state
        :param plan_state: Dict containing robot's current pose and some goal state
        :return: Dict containing motor speeds for each motor on the robot
        """
        curr_time = time.time()
        vehicle_commands = {
            'leftDriveMotorSpeed': 0,  # Left drive motor speed (-512 - 512)
            'rightDriveMotorSpeed': 0,  # Right drive motor speed (-512 - 512)
            'intakeCenterMotorSpeed': 0,  # Intake center motor speed (-512 - 512)
            'intakeLeftMotorSpeed': 0,  # Intake left motor speed (-512 - 512)
            'intakeRightMotorSpeed': 0,  # Intake right motor speed (-512 - 512)
            'tubeMotorSpeed': 0,  # Tube motor speed (-512 - 512)
            'timerStartStop': 0,  # Timer start/stop (0 or 1)
            'reset': 0,  # Reset (0 or 1)
            'outtake': 0,  # Outtake (0 or 1)
            'draw': []  # List of shapes to draw
        }

        if plan_state['flail']:
            curr_time = time.time()
            vehicle_commands['leftDriveMotorSpeed'] = int(self.max_forward_speed * math.sin(2 * curr_time))
            vehicle_commands['rightDriveMotorSpeed'] = int(self.max_forward_speed * math.sin(2.1 * curr_time))
            vehicle_commands['intakeCenterMotorSpeed'] = self.max_intake_speed
            vehicle_commands['intakeLeftMotorSpeed'] = self.max_intake_speed
            vehicle_commands['intakeRightMotorSpeed'] = self.max_intake_speed

        elif plan_state['trajectory'] is None:
            return vehicle_commands  # Nothing to do

        else:
            pose = plan_state['pose']
            start = np.array(plan_state['trajectory'][0])
            goal = np.array(plan_state['trajectory'][1])
            direction = plan_state['direction']
            tube_mode = plan_state['tube_mode']

            curr_heading = pose[1] % (2*np.pi)
            vec_start_to_goal = goal - start
            desired_heading = np.arctan2(vec_start_to_goal[1], vec_start_to_goal[0]) % (2*np.pi)

            if direction == -1:
                desired_heading += np.pi
                desired_heading = desired_heading % (2 * np.pi)

            # If we're not facing the right way, turn in place. Else move straight.
            heading_norm_vector = np.array([np.cos(curr_heading), np.sin(curr_heading)])
            goal_norm_vector = np.array([np.cos(desired_heading), np.sin(desired_heading)])
            x1 = heading_norm_vector[0]
            y1 = heading_norm_vector[1]
            x2 = goal_norm_vector[0]
            y2 = goal_norm_vector[1]
            heading_error = math.atan2(x1 * y2 - y1 * x2, x1 * x2 + y1 * y2)
            if direction == 0:
                left_drive_speed = 0
                right_drive_speed = 0
            elif abs(heading_error) >= self.heading_error_threshold:
                left_drive_speed = -int(self.left_drive_pid.run(0, heading_error, curr_time))
                right_drive_speed = int(self.right_drive_pid.run(0, heading_error, curr_time))
            else:
                distance_to_goal = geom.dist(start, goal)
                speed = int(abs(self.straight_pid.run(distance_to_goal, 0, curr_time)))
                speed = min(speed, self.max_forward_speed)
                left_drive_speed = speed * direction
                right_drive_speed = speed * direction

            intake_speed = self.max_intake_speed if tube_mode == 'INTAKE' else 0
            outtake_speed = self.max_outtake_speed if tube_mode == 'OUTTAKE' else 0

            vehicle_commands['leftDriveMotorSpeed'] = left_drive_speed
            vehicle_commands['rightDriveMotorSpeed'] = right_drive_speed
            vehicle_commands['intakeCenterMotorSpeed'] = intake_speed
            vehicle_commands['intakeLeftMotorSpeed'] = intake_speed
            vehicle_commands['intakeRightMotorSpeed'] = intake_speed
            vehicle_commands['tubeMotorSpeed'] = outtake_speed

        return vehicle_commands
