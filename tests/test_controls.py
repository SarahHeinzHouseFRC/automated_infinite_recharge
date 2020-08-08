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
        config.blue_player_station_pos = np.array([10, 10])

        self.controls = Controls(config)

        self.plan_state = {
            'pose': ((0, 0), 0),
            'trajectory': [(0, 0), (2.5, 0)],
            'direction': 0,
            'tube_mode': 'INTAKE',
            'flail': False,
        }

        self.vehicle_commands = {}

    def test_stand_still(self):
        actual = self.controls.run(self.plan_state)

        expected = {
            'leftDriveMotorSpeed': 0,
            'rightDriveMotorSpeed': 0,
            'intakeCenterMotorSpeed': self.controls.max_intake_speed,
            'intakeLeftMotorSpeed': self.controls.max_intake_speed,
            'intakeRightMotorSpeed': self.controls.max_intake_speed,
            'tubeMotorSpeed': 0,
            'timerStartStop': 0,
            'reset': 0,
            'outtake': 0,
            'draw': []
        }

        self.assertEqual(expected, actual)

    def test_drive_forward(self):
        self.plan_state['direction'] = 1

        actual = self.controls.run(self.plan_state)

        expected = {
            'leftDriveMotorSpeed': self.controls.max_forward_speed,
            'rightDriveMotorSpeed': self.controls.max_forward_speed,
            'intakeCenterMotorSpeed': self.controls.max_intake_speed,
            'intakeLeftMotorSpeed': self.controls.max_intake_speed,
            'intakeRightMotorSpeed': self.controls.max_intake_speed,
            'tubeMotorSpeed': 0,
            'timerStartStop': 0,
            'reset': 0,
            'outtake': 0,
            'draw': []
        }

        self.assertEqual(expected, actual)

    def test_drive_backward(self):
        self.plan_state['trajectory'] = [(0, 0), (-2.5, 0)]
        self.plan_state['direction'] = -1

        actual = self.controls.run(self.plan_state)

        expected = {
            'leftDriveMotorSpeed': -self.controls.max_forward_speed,
            'rightDriveMotorSpeed': -self.controls.max_forward_speed,
            'intakeCenterMotorSpeed': self.controls.max_intake_speed,
            'intakeLeftMotorSpeed': self.controls.max_intake_speed,
            'intakeRightMotorSpeed': self.controls.max_intake_speed,
            'tubeMotorSpeed': 0,
            'timerStartStop': 0,
            'reset': 0,
            'outtake': 0,
            'draw': []
        }

        self.assertEqual(expected, actual)

    def test_turn_right(self):
        self.plan_state['trajectory'] = [(0, 0), (0, -2.5)]
        self.plan_state['direction'] = 1

        vehicle_commands = self.controls.run(self.plan_state)

        self.assertTrue(vehicle_commands['leftDriveMotorSpeed'] > 0)
        self.assertTrue(vehicle_commands['rightDriveMotorSpeed'] < 0)

    def test_turn_left(self):
        self.plan_state['trajectory'] = [(0, 0), (0, 2.5)]
        self.plan_state['direction'] = 1

        vehicle_commands = self.controls.run(self.plan_state)

        self.assertTrue(vehicle_commands['leftDriveMotorSpeed'] < 0)
        self.assertTrue(vehicle_commands['rightDriveMotorSpeed'] > 0)

    def test_intake(self):
        self.plan_state['tube_mode'] = 'INTAKE'

        vehicle_commands = self.controls.run(self.plan_state)

        self.assertEqual(vehicle_commands['intakeCenterMotorSpeed'], self.controls.max_intake_speed)
        self.assertEqual(vehicle_commands['intakeLeftMotorSpeed'], self.controls.max_intake_speed)
        self.assertEqual(vehicle_commands['intakeRightMotorSpeed'], self.controls.max_intake_speed)
        self.assertEqual(vehicle_commands['tubeMotorSpeed'], 0)

    def test_outtake(self):
        self.plan_state['tube_mode'] = 'OUTTAKE'

        vehicle_commands = self.controls.run(self.plan_state)

        self.assertEqual(vehicle_commands['intakeCenterMotorSpeed'], 0)
        self.assertEqual(vehicle_commands['intakeLeftMotorSpeed'], 0)
        self.assertEqual(vehicle_commands['intakeRightMotorSpeed'], 0)
        self.assertEqual(vehicle_commands['tubeMotorSpeed'], self.controls.max_outtake_speed)

    def test_intake_running_during_flail(self):
        self.plan_state['flail'] = True

        vehicle_commands = self.controls.run(self.plan_state)

        self.assertEqual(vehicle_commands['intakeCenterMotorSpeed'], self.controls.max_intake_speed)
        self.assertEqual(vehicle_commands['intakeLeftMotorSpeed'], self.controls.max_intake_speed)
        self.assertEqual(vehicle_commands['intakeRightMotorSpeed'], self.controls.max_intake_speed)
        self.assertEqual(vehicle_commands['tubeMotorSpeed'], 0)

    def test_do_nothing_when_trajectory_is_none(self):
        self.plan_state['trajectory'] = None

        vehicle_commands = self.controls.run(self.plan_state)

        self.assertEqual(vehicle_commands['leftDriveMotorSpeed'], 0)
        self.assertEqual(vehicle_commands['rightDriveMotorSpeed'], 0)
        self.assertEqual(vehicle_commands['intakeCenterMotorSpeed'], 0)
        self.assertEqual(vehicle_commands['intakeLeftMotorSpeed'], 0)
        self.assertEqual(vehicle_commands['intakeRightMotorSpeed'], 0)
        self.assertEqual(vehicle_commands['tubeMotorSpeed'], 0)
