#!/usr/bin/env python3
"""
Вычисление локальных позиций точек опор в системе координат плеч.
Вынесено из InverseKinematics/robot_IK.py при декомпозиции.
"""

import numpy as np
from RoboticsUtilities.homogeneous_transforms import (
    homog_transform,
    homog_transform_inverse,
)


def compute_leg_transforms(body_length, body_width, dx, dy, dz, roll, pitch, yaw):
    """
    Вычислить матрицы преобразования для всех 4 ног.

    :return: Кортеж (T_blwFR1, T_blwFL1, T_blwRR1, T_blwRL1)
    """
    T_blwbl = homog_transform(dx, dy, dz, roll, pitch, yaw)

    T_blwFR1 = np.dot(
        T_blwbl,
        homog_transform(
            +0.5 * body_length, -0.5 * body_width, 0, np.pi / 2, -np.pi / 2, 0
        ),
    )
    T_blwFL1 = np.dot(
        T_blwbl,
        homog_transform(
            +0.5 * body_length, +0.5 * body_width, 0, np.pi / 2, -np.pi / 2, 0
        ),
    )
    T_blwRR1 = np.dot(
        T_blwbl,
        homog_transform(
            -0.5 * body_length, -0.5 * body_width, 0, np.pi / 2, -np.pi / 2, 0
        ),
    )
    T_blwRL1 = np.dot(
        T_blwbl,
        homog_transform(
            -0.5 * body_length, +0.5 * body_width, 0, np.pi / 2, -np.pi / 2, 0
        ),
    )

    return T_blwFR1, T_blwFL1, T_blwRR1, T_blwRL1


def compute_local_positions(
    leg_positions, body_length, body_width, dx, dy, dz, roll, pitch, yaw
):
    """
    Вычисление локальных позиций точек опор в системе координат плеч.

    :param leg_positions: Позиции ног (4x3 массив)
    :return: Локальные позиции (4x3 массив)
    """
    leg_positions = (np.block([[leg_positions], [np.array([1, 1, 1, 1])]])).T

    T_blwFR1, T_blwFL1, T_blwRR1, T_blwRL1 = compute_leg_transforms(
        body_length, body_width, dx, dy, dz, roll, pitch, yaw
    )

    pos_FR = np.dot(homog_transform_inverse(T_blwFR1), leg_positions[0])
    pos_FL = np.dot(homog_transform_inverse(T_blwFL1), leg_positions[1])
    pos_RR = np.dot(homog_transform_inverse(T_blwRR1), leg_positions[2])
    pos_RL = np.dot(homog_transform_inverse(T_blwRL1), leg_positions[3])

    return np.array([pos_FR[:3], pos_FL[:3], pos_RR[:3], pos_RL[:3]])
