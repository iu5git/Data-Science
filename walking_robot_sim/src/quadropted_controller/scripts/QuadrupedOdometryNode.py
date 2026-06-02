#!/usr/bin/env python3
"""
Узел одометрии четвероногого робота (тонкая обёртка над декомпозированными модулями).
"""

import rclpy
import tf2_ros

from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray

from quadropted_msgs.msg import RobotVelocity, RobotFootContact
from ForwardKinematics import ForwardKinematics

from QuadrupedOdometry import (
    OdometryState,
    NodeConfig,
    declare_parameters,
    SubscriptionCallbacks,
    OdometryPublisher,
    MarkerPublisher,
    MainLoop,
)


class DogOdometry(
    Node,
    SubscriptionCallbacks,
    OdometryPublisher,
    MarkerPublisher,
    MainLoop,
):
    """ROS-узел одометрии. Логика делегирована миксинам из QuadrupedOdometry.*"""

    def __init__(self):
        super().__init__("dog_odometry")

        # --- Конфигурация ---
        self.config: NodeConfig = declare_parameters(self)

        # Промоутим часто используемые параметры в атрибуты для удобства миксинов
        self.verbose = self.config.verbose
        self.publish_rate = self.config.publish_rate
        self.has_imu_heading = self.config.has_imu_heading
        self.enable_odom_tf = self.config.enable_odom_tf
        self.base_frame_id = self.config.base_frame_id
        self.odom_frame_id = self.config.odom_frame_id
        self.is_gazebo = self.config.is_gazebo

        # --- Состояние ---
        self.odom_state = OdometryState(
            filter_window_size=self.config.filter_window_size
        )
        self.last_position_time = self.get_clock().now()

        # --- Forward Kinematics ---
        self.fk_solver = ForwardKinematics(
            self.config.body_dimensions,
            self.config.leg_dimensions,
        )

        # --- Publishers ---
        self.odom_pub = self.create_publisher(
            Odometry, "odom", self.config.qos_reliable
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "foot_markers", self.config.qos_reliable
        )

        # --- Subscriptions ---
        if self.has_imu_heading:
            self.imu_sub = self.create_subscription(
                Imu,
                "imu_plugin/out",
                self.imu_callback,
                self.config.qos_reliable,
            )

        self.velocity_sub = self.create_subscription(
            RobotVelocity,
            "robot_velocity",
            self.velocity_callback,
            self.config.qos_reliable,
        )

        self.joint_states_sub = self.create_subscription(
            Float64MultiArray,
            "joint_group_controller/commands",
            self.joint_states_callback,
            self.config.qos_reliable,
        )

        self.foot_contacts_sub = self.create_subscription(
            RobotFootContact,
            "foot_contact",
            self.foot_contacts_callback,
            self.config.qos_best_effort,
        )

        # --- TF broadcaster ---
        if self.enable_odom_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Clock / encoder ---
        if self.is_gazebo:
            self.clock_sub = self.create_subscription(
                Clock,
                self.config.clock_topic,
                self.clock_callback,
                self.config.qos_reliable,
            )
            if self.verbose:
                self.get_logger().info("Subscribed to /clock topic with RELIABLE QoS.")
        else:
            encoder_qos = self.config.qos_best_effort
            self.encoder_sub = self.create_subscription(
                Int64,
                "encoder_value",
                self.encoder_callback,
                encoder_qos,
            )
            if self.verbose:
                self.get_logger().info(
                    "Subscribed to encoder_value topic with BEST_EFFORT QoS."
                )

        # --- Timer ---
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Dog Odometry Node has been started.")


def main(args=None):
    """Точка входа узла."""
    rclpy.init(args=args)
    node = None
    try:
        node = DogOdometry()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
