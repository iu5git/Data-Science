#!/usr/bin/env python3
"""
Конфигурация и параметры узла одометрии.
"""

from dataclasses import dataclass, field

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


@dataclass
class NodeConfig:
    """Параметры узла одометрии."""
    verbose: bool = False
    publish_rate: int = 50
    has_imu_heading: bool = True
    enable_odom_tf: bool = True
    base_frame_id: str = 'base'
    odom_frame_id: str = 'odom'
    is_gazebo: bool = True
    clock_topic: str = '/clock'

    # Размеры для Forward Kinematics
    body_dimensions: list = field(default_factory=lambda: [0.3762, 0.0935])
    leg_dimensions: list = field(default_factory=lambda: [0.0, 0.0955, 0.213, 0.213])

    # Фильтр одометрии
    filter_window_size: int = 14

    # Лимиты скоростей
    max_linear_velocity_x: float = 0.035
    max_linear_velocity_y: float = 0.012
    max_angular_velocity: float = 1.0

    # QoS профили
    @property
    def qos_reliable(self) -> QoSProfile:
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
            history=HistoryPolicy.KEEP_LAST
        )

    @property
    def qos_best_effort(self) -> QoSProfile:
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
            history=HistoryPolicy.KEEP_LAST
        )


def declare_parameters(node) -> NodeConfig:
    """Объявить и прочитать параметры узла.

    :param node: ROS Node для доступа к параметрам
    :return: NodeConfig с загруженными параметрами
    """
    config = NodeConfig()

    config.verbose = _param(node, 'verbose', config.verbose)
    config.publish_rate = _param(node, 'publish_rate', config.publish_rate)
    config.has_imu_heading = _param(node, 'has_imu_heading', config.has_imu_heading)
    config.enable_odom_tf = _param(node, 'enable_odom_tf', config.enable_odom_tf)
    config.base_frame_id = _param(node, 'base_frame_id', config.base_frame_id)
    config.odom_frame_id = _param(node, 'odom_frame_id', config.odom_frame_id)
    config.is_gazebo = _param(node, 'is_gazebo', config.is_gazebo)
    config.clock_topic = _param(node, 'clock_topic', config.clock_topic)

    if config.verbose:
        node.get_logger().info(f"Verbose mode: {config.verbose}")
        node.get_logger().info(f"Publish rate: {config.publish_rate} Hz")
        node.get_logger().info(f"Has IMU heading: {config.has_imu_heading}")
        node.get_logger().info(f"Enable odom TF: {config.enable_odom_tf}")
        node.get_logger().info(f"Base frame ID: {config.base_frame_id}")
        node.get_logger().info(f"Odom frame ID: {config.odom_frame_id}")
        node.get_logger().info(f"Is Gazebo: {config.is_gazebo}")
        node.get_logger().info(f"Clock Topic: {config.clock_topic}")

    return config


def _param(node, name: str, default):
    """Хелпер для объявления и чтения параметра."""
    node.declare_parameter(name, default)
    return node.get_parameter(name).value
