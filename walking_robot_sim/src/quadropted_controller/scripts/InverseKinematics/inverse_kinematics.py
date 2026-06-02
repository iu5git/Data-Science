#!/usr/bin/env python3
"""
Обратная кинематика квадрупеда.
Декомпозированная версия InverseKinematics/robot_IK.py.
"""

from .local_positions import compute_local_positions
from .joint_angles import compute_all_joint_angles


class InverseKinematics:
    def __init__(self, bodyDimensions, legDimensions):
        self.bodyLength = bodyDimensions[0]
        self.bodyWidth = bodyDimensions[1]

        self.l1 = legDimensions[0]
        self.l2 = legDimensions[1]
        self.l3 = legDimensions[2]
        self.l4 = legDimensions[3]

    def get_local_positions(self, leg_positions, dx, dy, dz, roll, pitch, yaw):
        """
        Вычисление локальных позиций точек опор в системе координат плеч.
        """
        return compute_local_positions(
            leg_positions, self.bodyLength, self.bodyWidth, dx, dy, dz, roll, pitch, yaw
        )

    def inverse_kinematics(self, leg_positions, dx, dy, dz, roll, pitch, yaw):
        """
        Вычисление обратной кинематики для всех ног.
        """
        positions = self.get_local_positions(
            leg_positions, dx, dy, dz, roll, pitch, yaw
        )
        return compute_all_joint_angles(positions, self.l1, self.l2, self.l3, self.l4)
