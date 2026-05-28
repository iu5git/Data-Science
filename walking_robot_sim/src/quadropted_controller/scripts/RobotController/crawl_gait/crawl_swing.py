#!/usr/bin/env python3
"""
Crawl Swing Controller — управление фазой подъёма ноги при походке crawl.
Вынесен из RobotController/CrawlGaitController.py при декомпозиции.
"""

import numpy as np
from RoboticsUtilities.rotation_matrices import rotz


class CrawlSwingController:
    def __init__(
        self,
        stance_ticks,
        swing_ticks,
        time_step,
        phase_length,
        z_leg_lift,
        default_stance,
        body_shift_y,
    ):
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.phase_length = phase_length
        self.z_leg_lift = z_leg_lift
        self.default_stance = default_stance
        self.body_shift_y = body_shift_y

    def raibert_touchdown_location(self, leg_index, command, shifted_left):
        delta_pos_2d = command.velocity * self.phase_length * self.time_step
        delta_pos = np.array([delta_pos_2d[0], delta_pos_2d[1], 0])
        theta = self.stance_ticks * self.time_step * command.yaw_rate
        rotation = rotz(theta)

        shift_correction = np.array([0.0, 0.0, 0.0])
        shift_correction[1] = -self.body_shift_y if shifted_left else self.body_shift_y

        return (
            np.matmul(rotation, self.default_stance[:, leg_index])
            + delta_pos
            + shift_correction
        )

    def swing_height(self, swing_phase):
        if swing_phase < 0.5:
            return swing_phase / 0.5 * self.z_leg_lift
        else:
            return self.z_leg_lift * (1 - (swing_phase - 0.5) / 0.5)

    def next_foot_location(self, swing_prop, leg_index, state, command, shifted_left):
        assert 0 <= swing_prop <= 1
        foot_location = state.foot_locations[:, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(
            leg_index, command, shifted_left
        )

        time_left = self.time_step * self.swing_ticks * (1.0 - swing_prop)
        velocity = (
            (touchdown_location - foot_location)
            / float(time_left)
            * np.array([1, 1, 0])
        )
        delta_foot_location = velocity * self.time_step
        z_vector = np.array([0, 0, swing_height_ + command.robot_height])

        return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location
