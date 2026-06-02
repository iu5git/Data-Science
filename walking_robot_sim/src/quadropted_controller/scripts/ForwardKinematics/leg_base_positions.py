#!/usr/bin/env python3
"""
Позиции баз ног робота.
Вынесено из ForwardKinematics/robot_FK.py при декомпозиции.
"""

import numpy as np


def get_leg_base_position(leg_index, body_length, body_width):
    """
    Получить базовую позицию (x, y) для ноги.

    :param leg_index: Индекс ноги (0: FR, 1: FL, 2: RR, 3: RL)
    :param body_length: Длина корпуса
    :param body_width: Ширина корпуса
    :return: Кортеж (base_x, base_y)
    """
    if leg_index == 0:  # FR
        return body_length / 2, body_width / 2
    elif leg_index == 1:  # FL
        return body_length / 2, -body_width / 2
    elif leg_index == 2:  # RR
        return -body_length / 2, body_width / 2
    elif leg_index == 3:  # RL
        return -body_length / 2, -body_width / 2
    else:
        raise ValueError(
            "Invalid leg_index. Must be 0 (FR), 1 (FL), 2 (RR), or 3 (RL)."
        )
