#!/usr/bin/env python3
"""
QuadrupedOdometry — пакет одометрии четвероногого робота.
Декомпозированная версия QuadrupedOdometryNode.py.
"""

from .odometry_state import OdometryState
from .odometry_update import update_odometry, normalize_angle
from .node_config import NodeConfig, declare_parameters
from .node_subscriptions import SubscriptionCallbacks
from .node_publishers import OdometryPublisher, MarkerPublisher
from .node_main import MainLoop

__all__ = [
    "OdometryState",
    "update_odometry",
    "normalize_angle",
    "NodeConfig",
    "declare_parameters",
    "SubscriptionCallbacks",
    "OdometryPublisher",
    "MarkerPublisher",
    "MainLoop",
]
