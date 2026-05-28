#!/usr/bin/env python3
"""
Вычисление углов суставов из локальных позиций ног.
Вынесено из InverseKinematics/robot_IK.py при декомпозиции.
"""

from math import sqrt, atan2, sin, cos, pi


def compute_joint_angles_for_leg(x, y, z, leg_index, l1, l2, l3, l4):
    """
    Вычислить углы суставов для одной ноги.

    :param x, y, z: Локальная позиция стопы
    :param leg_index: Индекс ноги (0-3)
    :param l1, l2, l3, l4: Размеры звеньев ноги
    :return: Кортеж (theta1, theta3, theta4)
    """
    F = sqrt(x**2 + y**2 - l2**2)
    G = F - l1
    H = sqrt(G**2 + z**2)

    theta1 = -atan2(y, x) - atan2(F, l2 * (-1) ** leg_index)

    D = (H**2 - l3**2 - l4**2) / (2 * l3 * l4)

    if D > 1:
        D = 1.0
    elif D < -1:
        D = -1.0

    theta4 = -atan2(sqrt(1 - D**2), D)

    theta3 = atan2(z, G) - atan2(l4 * sin(theta4), l3 + l4 * cos(theta4))

    return theta1, theta3, theta4


def compute_all_joint_angles(positions, l1, l2, l3, l4):
    """
    Вычислить углы суставов для всех 4 ног.

    :param positions: Массив локальных позиций (4x3)
    :return: Список из 12 углов (3 на ногу)
    """
    angles = []
    for i in range(4):
        x = positions[i][0]
        y = positions[i][1]
        z = positions[i][2]

        theta1, theta3, theta4 = compute_joint_angles_for_leg(
            x, y, z, i, l1, l2, l3, l4
        )
        angles.append(theta1)
        angles.append(theta3)
        angles.append(theta4)

    return angles
