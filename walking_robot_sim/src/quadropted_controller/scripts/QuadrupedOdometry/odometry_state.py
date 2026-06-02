#!/usr/bin/env python3
"""
Состояние одометрии четвероногого робота.
Вынесено из QuadrupedOdometryNode.py при декомпозиции.
"""

from collections import deque


class OdometryState:
    """Чистое состояние одометрии (без ROS зависимостей)."""

    def __init__(self, filter_window_size=14):
        # Позиция и ориентация
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Скорости
        self.linear_velocity_x = 0.0
        self.linear_velocity_y = 0.0
        self.imu_angular_velocity = 0.0

        # Фильтр скользящего среднего
        self.filter_window_size = filter_window_size
        self.delta_x_queue = deque(maxlen=filter_window_size)
        self.delta_y_queue = deque(maxlen=filter_window_size)

        # Позиции лап
        self.prev_foot_positions = [None, None, None, None]  # [FR, FL, RR, RL]
        self.foot_positions = [(0.0, 0.0, 0.0)] * 4

        # Контакты и суставы
        self.foot_contacts = [False, False, False, False]
        self.joint_positions = [0.0] * 12

        # Внешние данные
        self.gazebo_clock_sec = 0
        self.gazebo_clock_nanosec = 0
        self.encoder_pos = 0

    def reset(self):
        """Сбросить состояние к начальным значениям."""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity_x = 0.0
        self.linear_velocity_y = 0.0
        self.imu_angular_velocity = 0.0
        self.delta_x_queue.clear()
        self.delta_y_queue.clear()
        self.prev_foot_positions = [None, None, None, None]
        self.foot_positions = [(0.0, 0.0, 0.0)] * 4
        self.foot_contacts = [False, False, False, False]
        self.joint_positions = [0.0] * 12
