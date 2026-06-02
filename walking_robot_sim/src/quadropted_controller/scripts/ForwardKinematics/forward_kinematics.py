#!/usr/bin/env python3
"""
Прямая кинематика квадрупеда.
Декомпозированная версия ForwardKinematics/robot_FK.py.
"""

import numpy as np
from .leg_base_positions import get_leg_base_position
from .leg_fk_chain import compute_leg_fk_chain, build_homog_transform


class ForwardKinematics:
    def __init__(self, body_dimensions, leg_dimensions):
        """
        Инициализация параметров робота.
        :param body_dimensions: Размеры корпуса [длина, ширина]
        :param leg_dimensions: Размеры звеньев ног [l1, l2, l3, l4]
        """
        self.body_length = body_dimensions[0]
        self.body_width = body_dimensions[1]

        self.l1 = leg_dimensions[0]
        self.l2 = leg_dimensions[1]
        self.l3 = leg_dimensions[2]
        self.l4 = leg_dimensions[3]

    def homog_transform(self, dx, dy, dz, alpha, beta, gamma):
        """
        Создает однородную матрицу преобразования 4x4.
        Совместима с оригинальной версией из robot_FK.py.
        """
        return build_homog_transform(dx, dy, dz, alpha, beta, gamma)

    def forward_kinematics_per_leg(self, theta_hip, theta_thigh, theta_calf, leg_index):
        """
        Вычисление позиции лапы на основе углов суставов для одной ноги.
        """
        base_x, base_y = get_leg_base_position(
            leg_index, self.body_length, self.body_width
        )

        return compute_leg_fk_chain(
            theta_hip,
            theta_thigh,
            theta_calf,
            base_x,
            base_y,
            self.l1,
            self.l2,
            self.l3,
            self.l4,
        )

    def forward_kinematics_all_legs(self, joint_angles):
        """
        Вычисление позиций лап для всех ног.
        :param joint_angles: Список из 12 углов суставов
        :return: Список из 4 позиций лап
        """
        if len(joint_angles) != 12:
            raise ValueError("Expected 12 joint angles.")

        foot_positions = []
        for leg in range(4):
            idx = leg * 3
            theta_hip = joint_angles[idx]
            theta_thigh = joint_angles[idx + 1]
            theta_calf = joint_angles[idx + 2]

            foot_pos = self.forward_kinematics_per_leg(
                theta_hip, theta_thigh, theta_calf, leg
            )
            foot_positions.append(foot_pos)

        return foot_positions
