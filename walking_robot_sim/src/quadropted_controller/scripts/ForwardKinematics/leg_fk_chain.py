#!/usr/bin/env python3
"""
Цепочка кинематических преобразований для одной ноги.
Вынесено из ForwardKinematics/robot_FK.py при декомпозиции.
"""

import numpy as np
from math import sin, cos


def _build_rotation_x(alpha):
    """Однородная матрица вращения вокруг оси X (4x4)."""
    return np.array(
        [
            [1, 0, 0, 0],
            [0, cos(alpha), -sin(alpha), 0],
            [0, sin(alpha), cos(alpha), 0],
            [0, 0, 0, 1],
        ]
    )


def _build_rotation_y(beta):
    """Однородная матрица вращения вокруг оси Y (4x4)."""
    return np.array(
        [
            [cos(beta), 0, sin(beta), 0],
            [0, 1, 0, 0],
            [-sin(beta), 0, cos(beta), 0],
            [0, 0, 0, 1],
        ]
    )


def _build_rotation_z(gamma):
    """Однородная матрица вращения вокруг оси Z (4x4)."""
    return np.array(
        [
            [cos(gamma), -sin(gamma), 0, 0],
            [sin(gamma), cos(gamma), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )


def _build_translation(dx, dy, dz):
    """Однородная матрица трансляции (4x4)."""
    return np.array([[1, 0, 0, dx], [0, 1, 0, dy], [0, 0, 1, dz], [0, 0, 0, 1]])


def build_homog_transform(dx, dy, dz, alpha, beta, gamma):
    """
    Создает однородную матрицу преобразования 4x4.
    Совместима с версией из robot_FK.py.
    """
    trans = _build_translation(dx, dy, dz)
    rx = _build_rotation_x(alpha)
    ry = _build_rotation_y(beta)
    rz = _build_rotation_z(gamma)
    return trans @ rz @ ry @ rx


def compute_leg_fk_chain(
    theta_hip, theta_thigh, theta_calf, base_x, base_y, l1, l2, l3, l4
):
    """
    Вычислить позицию стопы одной ноги через цепочку преобразований.

    :param theta_hip: Угол hip_joint (радианы)
    :param theta_thigh: Угол thigh_joint (радианы)
    :param theta_calf: Угол calf_joint (радианы)
    :param base_x: Базовая позиция X ноги
    :param base_y: Базовая позиция Y ноги
    :param l1: Высота корпуса
    :param l2: Длина бедра
    :param l3: Длина голени
    :param l4: Длина копыта
    :return: Позиция стопы (x, y, z)
    """
    T_base = build_homog_transform(base_x, base_y, -l1, 0, 0, 0)
    T_hip_abd = build_homog_transform(0, 0, 0, 0, 0, theta_hip)
    T_thigh_pitch = build_homog_transform(0, 0, 0, 0, theta_thigh, 0)
    T_thigh = build_homog_transform(l2, 0, 0, 0, 0, 0)
    T_calf_pitch = build_homog_transform(0, 0, 0, 0, theta_calf, 0)
    T_calf = build_homog_transform(l3, 0, 0, 0, 0, 0)
    T_foot = build_homog_transform(l4, 0, 0, 0, 0, 0)

    T_total = (
        T_base @ T_hip_abd @ T_thigh_pitch @ T_thigh @ T_calf_pitch @ T_calf @ T_foot
    )

    foot_position = T_total @ np.array([0, 0, 0, 1])
    return foot_position[:3]
