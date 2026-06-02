#!/usr/bin/env python3
"""
Модуль матриц вращения (3x3).
Вынесен из RoboticsUtilities/Transformations.py при декомпозиции.
"""

import numpy as np
from math import sin, cos


def rotx(alpha):
    """
    Создать матрицу вращения 3x3 вокруг оси X.
    """
    rx = np.array(
        [
            [
                1,
                0,
                0,
            ],
            [0, cos(alpha), -sin(alpha)],
            [0, sin(alpha), cos(alpha)],
        ]
    )
    return rx


def roty(beta):
    """
    Создать матрицу вращения 3x3 вокруг оси Y.
    """
    ry = np.array([[cos(beta), 0, sin(beta)], [0, 1, 0], [-sin(beta), 0, cos(beta)]])
    return ry


def rotz(gamma):
    """
    Создать матрицу вращения 3x3 вокруг оси Z.
    """
    rz = np.array(
        [
            [np.cos(gamma), -np.sin(gamma), 0],
            [np.sin(gamma), np.cos(gamma), 0],
            [0, 0, 1],
        ]
    )
    return rz


def rotxyz(alpha, beta, gamma):
    """
    Создать матрицу вращения 3x3 вокруг осей X, Y, Z (последовательно).
    """
    return rotx(alpha).dot(roty(beta)).dot(rotz(gamma))
