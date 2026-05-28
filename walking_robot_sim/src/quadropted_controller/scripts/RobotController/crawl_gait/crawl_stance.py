#!/usr/bin/env python3
"""
Crawl Stance Controller — управление фазой опоры ноги при походке crawl.
Вынесен из RobotController/CrawlGaitController.py при декомпозиции.
"""

import numpy as np
from RoboticsUtilities.rotation_matrices import rotz


class CrawlStanceController:
    def __init__(
        self,
        phase_length,
        stance_ticks,
        swing_ticks,
        time_step,
        z_error_constant,
        body_shift_y,
    ):
        self.phase_length = phase_length
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.z_error_constant = z_error_constant
        self.body_shift_y = body_shift_y

    def position_delta(
        self, leg_index, state, command, first_cycle, move_sideways, move_left
    ):
        z = state.foot_locations[2, leg_index]
        step_dist_x = command.velocity[0] * (
            float(self.phase_length) / self.swing_ticks
        )
        shift_factor = 1 if first_cycle else 2

        side_vel = 0.0
        if move_sideways:
            side_vel = (
                -(self.body_shift_y * shift_factor)
                / (float(self.time_step) * self.stance_ticks)
                if move_left
                else (self.body_shift_y * shift_factor)
                / (float(self.time_step) * self.stance_ticks)
            )

        velocity = np.array(
            [
                -(step_dist_x / 3) / (float(self.time_step) * self.stance_ticks),
                side_vel,
                1.0 / self.z_error_constant * (state.robot_height - z),
            ]
        )

        delta_pos = velocity * self.time_step
        delta_ori = rotz(-command.yaw_rate * self.time_step)

        return delta_pos, delta_ori

    def next_foot_location(
        self, leg_index, state, command, first_cycle, move_sideways, move_left
    ):
        foot_location = state.foot_locations[:, leg_index]
        delta_pos, delta_ori = self.position_delta(
            leg_index, state, command, first_cycle, move_sideways, move_left
        )
        return np.matmul(delta_ori, foot_location) + delta_pos
