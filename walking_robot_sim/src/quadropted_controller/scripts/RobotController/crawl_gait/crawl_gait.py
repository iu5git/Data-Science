#!/usr/bin/env python3
"""
Crawl Gait Controller — главный контроллер походки crawl.
Декомпозированная версия RobotController/CrawlGaitController.py.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from RoboticsUtilities.rotation_matrices import rotz
from ..GaitController import GaitController
from .crawl_swing import CrawlSwingController
from .crawl_stance import CrawlStanceController


class CrawlGaitController(GaitController):
    def __init__(self, default_stance, stance_time, swing_time, time_step):
        contact_phases = np.array([[1, 1, 1, 0, 1, 1, 1, 1],    # 0: leg swing
                                   [1, 1, 1, 1, 1, 1, 1, 0],    # 1: Moving stance forward
                                   [1, 0, 1, 1, 1, 1, 1, 1],
                                   [1, 1, 1, 1, 1, 0, 1, 1]])

        z_error_constant = 0.02     # Determines how fast we move toward the goal in the z axis
        z_leg_lift = 0.14

        super().__init__(stance_time, swing_time, time_step, contact_phases, default_stance)
        self.max_x_velocity = 0.011 #[m/s]
        self.max_yaw_rate = 0.15 #[rad/s]
        self.body_shift_y = 0.06

        self.swingController = CrawlSwingController(self.stance_ticks, self.swing_ticks, self.time_step,
                                                    self.phase_length, z_leg_lift, self.default_stance, self.body_shift_y)
        self.stanceController = CrawlStanceController(self.phase_length, self.stance_ticks, self.swing_ticks,
                                                      self.time_step, z_error_constant, self.body_shift_y)

        self.first_cycle = True

    def updateStateCommand(self, msg, state, command):
        command.velocity[0] = msg.axes[4] * self.max_x_velocity
        command.yaw_rate = msg.axes[0] * self.max_yaw_rate

    def step(self, state, command):
        contact_modes = self.contacts(state.ticks)
        new_foot_locations = np.zeros((3,4))
        phase_index = self.phase_index(state.ticks)

        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            if contact_mode == 1:
                if phase_index in (0,4):
                    move_sideways = True
                    move_left = (phase_index == 0)
                else:
                    move_sideways = False
                    move_left = False

                new_location = self.stanceController.next_foot_location(leg_index, state, command,
                                                                        self.first_cycle, move_sideways, move_left)
            else:
                swing_proportion = float(self.subphase_ticks(state.ticks)) / float(self.swing_ticks)
                shifted_left = (phase_index in (1,3))

                new_location = self.swingController.next_foot_location(swing_proportion, leg_index, state,
                                                                       command, shifted_left)

            new_foot_locations[:, leg_index] = new_location

        return new_foot_locations

    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        state.ticks += 1
        state.robot_height = command.robot_height

        if self.phase_index(state.ticks) > 0 and self.first_cycle:
            self.first_cycle = False

        return state.foot_locations
