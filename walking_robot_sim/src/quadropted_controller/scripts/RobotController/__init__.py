#!/usr/bin/env python3
"""
RobotController — пакет контроллеров робота.
"""

from .GaitController import GaitController
from .PIDController import PID_controller
from .StateCommand import State, Command, BehaviorState
from .StandController import StandController
from .RestController import RestController
from .trot_gait.trot_gait import TrotGaitController
from .crawl_gait.crawl_gait import CrawlGaitController
from .RobotController import Robot

__all__ = [
    "GaitController",
    "PID_controller",
    "State",
    "Command",
    "BehaviorState",
    "StandController",
    "RestController",
    "TrotGaitController",
    "CrawlGaitController",
    "Robot",
]
