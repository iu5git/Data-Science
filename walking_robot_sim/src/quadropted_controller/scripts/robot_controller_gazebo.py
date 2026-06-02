#!/usr/bin/env python3
# Author: lnotspotl, abutalipovvv
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from quadropted_msgs.msg import RobotModeCommand, RobotVelocity
from quadropted_msgs.srv import RobotBehaviorCommand
from InverseKinematics import InverseKinematics
from RobotController import Robot
import numpy as np

USE_IMU = True
RATE = 60
DEBUG_JOINTS = True  # Включить логирование суставов для отладки (аналог C++ DEBUG)


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__("Robot_Controller")

        # Объявляем параметры
        self.declare_parameter("verbose", False)
        self.verbose = self.get_parameter("verbose").get_parameter_value().bool_value
        if self.verbose:
            self.get_logger().info(f"Verbose mode: {self.verbose}")

        # Геометрия робота
        body = [0.3762, 0.0935]
        legs = [0.0, 0.0955, 0.213, 0.213]

        self.declare_parameter("robot_id", 1)
        self.robot_id = (
            self.get_parameter("robot_id").get_parameter_value().integer_value
        )

        self.robot = Robot(self, body, legs, USE_IMU, self.robot_id)
        self.inverseKinematics = InverseKinematics(body, legs)

        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray, "joint_group_controller/commands", 10
        )

        self.create_subscription(
            RobotModeCommand, "robot_mode", self.robot.mode_callback, 10
        )
        self.create_subscription(
            RobotVelocity, "robot_velocity", self.robot.velocity_callback, 10
        )

        self.timer = self.create_timer(1.0 / RATE, self.control_loop)

        # Переиспользуемое сообщение для команд суставам
        self._joint_command_msg = Float64MultiArray()

        # Флаг для ленивой смены контроллера (вызывается только при изменении)
        self._controller_change_needed = False
        
        # Счётчик тиков для debug логов (каждые 60 тиков = 1 сек)
        self._debug_tick_count = 0

    def control_loop(self):
        leg_positions = self.robot.run()

        if self._controller_change_needed:
            self.robot.change_controller()
            self._controller_change_needed = False

        dx = self.robot.state.body_local_position[0]
        dy = self.robot.state.body_local_position[1]
        dz = self.robot.state.body_local_position[2]

        roll = self.robot.state.body_local_orientation[0]
        pitch = self.robot.state.body_local_orientation[1]
        yaw = self.robot.state.body_local_orientation[2]

        try:
            cmd = self.robot.command
            joint_angles = self.inverseKinematics.inverse_kinematics(
                leg_positions, dx, dy, dz, roll, pitch, yaw
            )

            self._joint_command_msg.data = joint_angles
            self.joint_command_publisher.publish(self._joint_command_msg)

            # DEBUG: логирование каждые 60 тиков (аналогично C++ версии)
            if DEBUG_JOINTS:
                self._debug_tick_count += 1
                if self._debug_tick_count % 60 == 0:
                    self.get_logger().info(
                        f"[DEBUG] cmd: vx={cmd.velocity[0]:.4f} vy={cmd.velocity[1]:.4f} "
                        f"vz={cmd.velocity[2]:.4f} yaw={cmd.yaw_rate[2]:.4f} | "
                        f"pos: x={dx:.4f} y={dy:.4f} z={dz:.4f} | "
                        f"joints[0-2]: {joint_angles[0]:.4f} {joint_angles[1]:.4f} {joint_angles[2]:.4f}"
                    )

            if self.verbose:
                self.get_logger().info(f"Published joint angles: {joint_angles}")

        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
