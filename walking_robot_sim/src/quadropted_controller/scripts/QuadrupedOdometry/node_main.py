#!/usr/bin/env python3
"""
Основной цикл узла (timer_callback, FK, update_odometry).
"""

from QuadrupedOdometry import update_odometry


class MainLoop:
    """Миксин с основным циклом (timer_callback, FK, update_odometry)."""

    def calculate_foot_positions(self):
        """Вычислить позиции лап через FK."""
        if len(self.odom_state.joint_positions) != 12:
            self.get_logger().error(
                f"Incorrect number of joint positions: {len(self.odom_state.joint_positions)}. Expected 12."
            )
            return

        try:
            foot_positions = self.fk_solver.forward_kinematics_all_legs(
                self.odom_state.joint_positions
            )
            self.odom_state.foot_positions = foot_positions
        except Exception as e:
            self.get_logger().error(f"Error in forward kinematics: {e}")
            self.odom_state.foot_positions = [(0.0, 0.0, 0.0)] * 4
            return

        if self.verbose:
            for i, pos in enumerate(self.odom_state.foot_positions):
                leg = ['FR', 'FL', 'RR', 'RL'][i]
                self.get_logger().info(
                    f"{leg} Foot Position: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}"
                )

    def update_odometry_step(self):
        """Обновить одометрию (делегирование к чистой функции)."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_position_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        update_odometry(self.odom_state, dt)

        if self.verbose:
            self.get_logger().info(
                f"Odometry updated: x={self.odom_state.x:.6f}, "
                f"y={self.odom_state.y:.6f}, theta={self.odom_state.theta:.6f}"
            )

        self.last_position_time = current_time

    def timer_callback(self):
        """Основной цикл: FK -> одометрия -> публикация -> маркеры."""
        self.calculate_foot_positions()
        self.update_odometry_step()
        self.publish_odometry()
        self.publish_markers()

        if self.verbose:
            self.get_logger().info(
                f"Position Updated: x={self.odom_state.x:.6f} m, "
                f"y={self.odom_state.y:.6f} m, "
                f"theta={self.odom_state.theta:.6f} rad"
            )
