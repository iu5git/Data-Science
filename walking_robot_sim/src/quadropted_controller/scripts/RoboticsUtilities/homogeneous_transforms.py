#!/usr/bin/env python3
"""
Модуль однородных преобразований (4x4).
Вынесен из RoboticsUtilities/Transformations.py при декомпозиции.
"""

import numpy as np
from .rotation_matrices import rotxyz


def homog_transxyz(dx, dy, dz):
    """
    Создать однородную матрицу трансляции 4x4 (перемещение по осям x, y, z).
    """
    trans = np.array([[1, 0, 0, dx], [0, 1, 0, dy], [0, 0, 1, dz], [0, 0, 0, 1]])
    return trans


def homog_transform(dx, dy, dz, alpha, beta, gamma):
    """
    Создать однородную матрицу преобразования 4x4.
    """
    rot4x4 = np.eye(4)
    rot4x4[:3, :3] = rotxyz(alpha, beta, gamma)
    return np.dot(homog_transxyz(dx, dy, dz), rot4x4)


def homog_transform_inverse(matrix):
    """
    Вернуть инверсию однородной матрицы преобразования.

                 -------------------------
                 |           |           |
    inverse   =  |    R^T    |  -R^T * d |
                 |___________|___________|
                 | 0   0   0 |     1     |
                 -------------------------

    """
    inverse = matrix
    inverse[:3, :3] = inverse[:3, :3].T  # R^T
    inverse[:3, 3] = -np.dot(inverse[:3, :3], inverse[:3, 3])  # -R^T * d
    return inverse
