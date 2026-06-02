#!/usr/bin/env python3
"""
Подписки и callback'и узла одометрии.
"""

import tf_transformations


class SubscriptionCallbacks:
    """Миксин с callback'ами для подписок.

    Используется через множественное наследование в DogOdometry.
    """

    # --- velocity ---

    def velocity_callback(self, msg):
        self.odom_state.linear_velocity_x = msg.cmd_vel.linear.x
        self.odom_state.linear_velocity_y = msg.cmd_vel.linear.y
        if self.verbose:
            self.get_logger().info(
                f"Robot Velocity - Linear X: {self.odom_state.linear_velocity_x:.6f} m/s, "
                f"Linear Y: {self.odom_state.linear_velocity_y:.6f} m/s"
            )

    # --- joint states ---

    def joint_states_callback(self, msg):
        if len(msg.data) != 12:
            self.get_logger().error(
                f"Unexpected number of joint angles: {len(msg.data)}. Expected 12."
            )
            return
        self.odom_state.joint_positions = list(msg.data)
        if self.verbose:
            self.get_logger().info(f"Joint Positions: {self.odom_state.joint_positions}")

    # --- foot contacts ---

    def foot_contacts_callback(self, msg):
        if self.verbose:
            self.get_logger().info(f"Received foot_contacts message: {msg}")

        if len(msg.contacts) != 4:
            self.get_logger().error(
                f"Unexpected number of contacts: {len(msg.contacts)}. Expected 4."
            )
            self.odom_state.foot_contacts = [False, False, False, False]
            return

        self.odom_state.foot_contacts = list(msg.contacts)
        if self.verbose:
            self.get_logger().info(f"Foot Contacts: {self.odom_state.foot_contacts}")

    # --- IMU ---

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)

        self.odom_state.theta = yaw
        self.odom_state.imu_angular_velocity = -msg.angular_velocity.z

        if self.verbose:
            self.get_logger().info(f"IMU Yaw: {self.odom_state.theta:.6f} rad")
            self.get_logger().info(f"IMU Angular Velocity: {self.odom_state.imu_angular_velocity:.6f} rad/s")

    # --- clock (Gazebo) ---

    def clock_callback(self, msg):
        self.odom_state.gazebo_clock_sec = msg.clock.sec
        self.odom_state.gazebo_clock_nanosec = msg.clock.nanosec
        if self.verbose:
            self.get_logger().info(
                f"Received Gazebo Clock: {self.odom_state.gazebo_clock_sec}.{self.odom_state.gazebo_clock_nanosec}"
            )

    # --- encoder (hardware) ---

    def encoder_callback(self, msg):
        self.odom_state.encoder_pos = msg.data
        if self.verbose:
            self.get_logger().info(f"Received Encoder Position: {self.odom_state.encoder_pos}")
