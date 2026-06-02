#!/usr/bin/env python3
"""
Trot Gait Controller — главный контроллер походки trot.
Декомпозированная версия RobotController/TrotGaitController.py.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from RoboticsUtilities.rotation_matrices import rotxyz, rotz
from ..GaitController import GaitController
from ..PIDController import PID_controller
from geometry_msgs.msg import Twist
from quadropted_msgs.msg import RobotFootContact
from .trot_swing import TrotSwingController
from .trot_stance import TrotStanceController


class TrotGaitController(GaitController):
    def __init__(self, node, default_stance, stance_time, swing_time, time_step, use_imu):
        self.node = node
        self.use_imu = use_imu
        self.use_button = True
        self.autoRest = True
        self.trotNeeded = True


        contact_phases = np.array([[1, 1, 1, 0],  # 0: Leg swing
                                   [1, 0, 1, 1],  # 1: Moving stance forward
                                   [1, 0, 1, 1],
                                   [1, 1, 1, 0]])

        z_error_constant = 0.02  # This constant determines how fast we move
                                 # toward the goal in the z direction
        z_leg_lift = 0.14

        super().__init__(stance_time, swing_time, time_step, contact_phases, default_stance)

        self.velocity_pub = self.node.create_publisher(Twist, "controller_velocity", 10)

        self.foot_contact_pub = self.node.create_publisher(RobotFootContact, "foot_contact", 10)

        self.max_x_velocity = 0.035  # [m/s]
        self.max_y_velocity = 0.012  # [m/s]
        self.max_yaw_rate = 0.5  # [rad/s]

        self.swingController = TrotSwingController(
            self.stance_ticks,
            self.swing_ticks,
            self.time_step,
            self.phase_length,
            z_leg_lift,
            self.default_stance
        )

        self.stanceController = TrotStanceController(
            self.phase_length,
            self.stance_ticks,
            self.swing_ticks,
            self.time_step,
            z_error_constant
        )

        # TODO: tune kp, ki and kd
        #                                     kp    ki    kd
        self.pid_controller = PID_controller(0.15, 0.02, 0.002)

    def updateStateCommand(self, msg, state, command):
        command.velocity[0] = msg.axes[4] * self.max_x_velocity
        command.velocity[1] = msg.axes[3] * self.max_y_velocity
        command.yaw_rate[2] = msg.axes[0] * self.max_yaw_rate


        velocity_msg = Twist()
        velocity_msg.linear.x = command.velocity[0]
        velocity_msg.linear.y = command.velocity[1]
        velocity_msg.angular.z = command.yaw_rate[2]
        self.velocity_pub.publish(velocity_msg)

        velocity_msg_raw = Twist()
        velocity_msg_raw.linear.x = msg.axes[4] * 0.5
        velocity_msg_raw.linear.y = msg.axes[3] * 0.5
        velocity_msg_raw.angular.z = msg.axes[0]
        self.velocity_pub.publish(velocity_msg_raw)

        if self.use_button:
            if msg.buttons[7]:
                self.use_imu = not self.use_imu
                self.use_button = False
                self.node.get_logger().info(f"Trot Gait Controller - Use roll/pitch compensation: {self.use_imu}")

            elif msg.buttons[6]:
                self.autoRest = not self.autoRest
                if not self.autoRest:
                    self.trotNeeded = True
                self.use_button = False
                self.node.get_logger().info(f"Trot Gait Controller - Use autorest: {self.autoRest}")

        if not self.use_button:
            if not (msg.buttons[6] or msg.buttons[7]):
                self.use_button = True

    def step(self, state, command):
        if self.autoRest:
            if command.velocity[0] == 0 and command.velocity[1] == 0 and np.all(command.yaw_rate == 0):
                if state.ticks % (2 * self.phase_length) == 0:
                    self.trotNeeded = False
            else:
                self.trotNeeded = True

        if self.trotNeeded:
            contact_modes = self.contacts(state.ticks)

            foot_contact_msg = RobotFootContact()
            foot_contact_msg.contacts = [bool(mode) for mode in contact_modes.tolist()]
            self.foot_contact_pub.publish(foot_contact_msg)

            new_foot_locations = np.zeros((3, 4))
            for leg_index in range(4):
                contact_mode = contact_modes[leg_index]
                if contact_mode == 1:
                    new_location = self.stanceController.next_foot_location(leg_index, state, command)
                else:
                    swing_proportion = float(self.subphase_ticks(state.ticks)) / float(self.swing_ticks)

                    new_location = self.swingController.next_foot_location(swing_proportion, leg_index, state, command)

                new_foot_locations[:, leg_index] = new_location

            # Компенсация крена и тангажа
            if self.use_imu:
                compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)
                roll_compensation = -compensation[0]
                pitch_compensation = -compensation[1]

                rot = rotxyz(roll_compensation, pitch_compensation, 0)
                new_foot_locations = np.matmul(rot, new_foot_locations)

            state.ticks += 1
            return new_foot_locations
        else:
            temp = self.default_stance.copy()
            temp[2] = [command.robot_height] * 4

            foot_contact_msg = RobotFootContact()
            foot_contact_msg.contacts = [True, True, True, True]  # [FR, FL, RR, RL]
            self.foot_contact_pub.publish(foot_contact_msg)

            return temp

    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        state.robot_height = command.robot_height

        return state.foot_locations
