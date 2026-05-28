#!/usr/bin/env python3
"""
Trot Stance Controller — управление фазой опоры ноги при походке trot.
Вынесен из RobotController/TrotGaitController.py при декомпозиции.
"""

import numpy as np
from RoboticsUtilities.rotation_matrices import rotxyz


class TrotStanceController:
    def __init__(
        self, phase_length, stance_ticks, swing_ticks, time_step, z_error_constant
    ):
        self.phase_length = phase_length
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.z_error_constant = z_error_constant

    def position_delta(self, leg_index, state, command):
        z = state.foot_locations[2, leg_index]

        step_dist_x = command.velocity[0] * (
            float(self.phase_length) / self.swing_ticks
        )
        step_dist_y = command.velocity[1] * (
            float(self.phase_length) / self.swing_ticks
        )

        velocity = np.array(
            [
                -(step_dist_x / 4) / (float(self.time_step) * self.stance_ticks),
                -(step_dist_y / 4) / (float(self.time_step) * self.stance_ticks),
                1.0 / self.z_error_constant * (state.robot_height - z),
            ]
        )

        delta_pos = velocity * self.time_step
        delta_ori = rotxyz(
            -command.yaw_rate[0] * self.time_step,
            -command.yaw_rate[1] * self.time_step,
            -command.yaw_rate[2] * self.time_step,
        )
        return (delta_pos, delta_ori)

    def next_foot_location(self, leg_index, state, command):
        foot_location = state.foot_locations[:, leg_index]
        (delta_pos, delta_ori) = self.position_delta(leg_index, state, command)
        next_foot_location = np.matmul(delta_ori, foot_location) + delta_pos
        return next_foot_location
