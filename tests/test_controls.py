#
# Copyright (c) 2020 FRC Team 3260
#

import unittest
from unittest.mock import Mock
from controls import Controls, PID
import numpy as np


class TestPID(unittest.TestCase):
    def setUp(self):
        self.pid = PID(1, 1, 1)

    def test_gain_is_high_when_desired_higher_than_curr_value(self):
        gain = self.pid.run(curr_value=0, desired_value=100, curr_timestamp=0)
        self.assertTrue(gain > 0)

    def test_gain_is_low_when_desired_lower_than_curr_value(self):
        gain = self.pid.run(curr_value=0, desired_value=-100, curr_timestamp=0)
        self.assertTrue(gain < 0)

    def test_gain_is_zero_when_desired_equals_curr_value(self):
        gain = self.pid.run(curr_value=0, desired_value=0, curr_timestamp=0)
        self.assertTrue(gain == 0)


class TestControls(unittest.TestCase):
    def setUp(self):
        config = Mock()
        config.drive_kp = 40
        config.drive_ki = 1
        config.drive_kd = 1
        config.heading_error_threshold = 0.2
        config.max_forward_speed = 512
        config.max_intake_speed = 512
        config.max_outtake_speed = 512

        self.controls = Controls(config)

        self.plan_state = {
            'pose': ((0, 0), 0),
            'trajectory': None,
            'tube_mode': 'NONE',
            'direction': 0,
            'field_outtake': 0,
            'flail': False,
            'goal': None,
        }

        self.vehicle_commands = {}

    def test_do_nothing_when_trajectory_is_none(self):
        self.plan_state['trajectory'] = None

        vehicle_commands = self.controls.run(self.plan_state)

        self.assertEqual(vehicle_commands['leftDriveMotorSpeed'], 0)
        self.assertEqual(vehicle_commands['rightDriveMotorSpeed'], 0)
        self.assertEqual(vehicle_commands['intakeCenterMotorSpeed'], 0)
        self.assertEqual(vehicle_commands['intakeLeftMotorSpeed'], 0)
        self.assertEqual(vehicle_commands['intakeRightMotorSpeed'], 0)
        self.assertEqual(vehicle_commands['tubeMotorSpeed'], 0)

    def test_stand_still(self):
        actual = self.controls.run(self.plan_state)

        expected = {
            'leftDriveMotorSpeed': 0,
            'rightDriveMotorSpeed': 0,
            'intakeCenterMotorSpeed': 0,
            'intakeLeftMotorSpeed': 0,
            'intakeRightMotorSpeed': 0,
            'tubeMotorSpeed': 0,
            'timerStartStop': 0,
            'reset': 0,
            'outtake': 0,
            'draw': []
        }

        self.assertEqual(expected, actual)

    def test_drive_forward_when_commanded_direction_is_forward(self):
        self.plan_state['trajectory'] = [(0, 0), (2.5, 0)]
        self.plan_state['direction'] = 1

        actual = self.controls.run(self.plan_state)

        expected = {
            'leftDriveMotorSpeed': self.controls.max_forward_speed,
            'rightDriveMotorSpeed': self.controls.max_forward_speed,
            'intakeCenterMotorSpeed': 0,
            'intakeLeftMotorSpeed': 0,
            'intakeRightMotorSpeed': 0,
            'tubeMotorSpeed': 0,
            'timerStartStop': 0,
            'reset': 0,
            'outtake': 0,
            'draw': []
        }

        self.assertEqual(expected, actual)

    def test_drive_backward_when_commanded_direction_is_backward(self):
        self.plan_state['trajectory'] = [(0, 0), (-2.5, 0)]
        self.plan_state['direction'] = -1

        actual = self.controls.run(self.plan_state)

        expected = {
            'leftDriveMotorSpeed': -self.controls.max_forward_speed,
            'rightDriveMotorSpeed': -self.controls.max_forward_speed,
            'intakeCenterMotorSpeed': 0,
            'intakeLeftMotorSpeed': 0,
            'intakeRightMotorSpeed': 0,
            'tubeMotorSpeed': 0,
            'timerStartStop': 0,
            'reset': 0,
            'outtake': 0,
            'draw': []
        }

        self.assertEqual(expected, actual)

    def test_turns_right_when_waypoint_is_to_the_right(self):
        self.plan_state['trajectory'] = [(0, 0), (0, -2.5)]
        self.plan_state['direction'] = 1

        vehicle_commands = self.controls.run(self.plan_state)

        self.assertTrue(vehicle_commands['leftDriveMotorSpeed'] > 0)
        self.assertTrue(vehicle_commands['rightDriveMotorSpeed'] < 0)

    def test_turns_left_when_waypoint_is_to_the_left(self):
        self.plan_state['trajectory'] = [(0, 0), (0, 2.5)]
        self.plan_state['direction'] = 1

        vehicle_commands = self.controls.run(self.plan_state)

        self.assertTrue(vehicle_commands['leftDriveMotorSpeed'] < 0)
        self.assertTrue(vehicle_commands['rightDriveMotorSpeed'] > 0)

    def test_runs_intake_at_full_speed_when_given_tube_intake_command(self):
        self.plan_state['tube_mode'] = 'INTAKE'

        vehicle_commands = self.controls.run(self.plan_state)

        self.assertEqual(vehicle_commands['intakeCenterMotorSpeed'], self.controls.max_intake_speed)
        self.assertEqual(vehicle_commands['intakeLeftMotorSpeed'], self.controls.max_intake_speed)
        self.assertEqual(vehicle_commands['intakeRightMotorSpeed'], self.controls.max_intake_speed)
        self.assertEqual(vehicle_commands['tubeMotorSpeed'], 0)

    def test_runs_tube_at_full_speed_when_given_tube_outtake_command(self):
        self.plan_state['tube_mode'] = 'OUTTAKE'

        vehicle_commands = self.controls.run(self.plan_state)

        self.assertEqual(vehicle_commands['intakeCenterMotorSpeed'], 0)
        self.assertEqual(vehicle_commands['intakeLeftMotorSpeed'], 0)
        self.assertEqual(vehicle_commands['intakeRightMotorSpeed'], 0)
        self.assertEqual(vehicle_commands['tubeMotorSpeed'], self.controls.max_outtake_speed)

    # TODO: Make flail more testable

    def test_field_outtake_toggles_when_commanded_outtake(self):
        self.plan_state['field_outtake'] = True

        vehicle_commands = self.controls.run(self.plan_state)

        self.assertEqual(vehicle_commands['outtake'], 1)
