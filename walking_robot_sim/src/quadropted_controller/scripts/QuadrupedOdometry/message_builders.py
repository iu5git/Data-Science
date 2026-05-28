#!/usr/bin/env python3
"""
Построение сообщений одометрии и TF — чистые функции (без ROS зависимостей).
Вынесено из QuadrupedOdometryNode.py при декомпозиции.

Эти функции принимают все данные как аргументы и возвращают словари
с данными, которые затем используются ROS нодой для создания сообщений.
"""

import math


def build_odometry_data(x, y, theta, linear_vx, linear_vy, angular_vz,
                        frame_id, child_frame_id, stamp):
    """
    Построить данные одометрии.

    :return: dict с полями для Odometry message
    """
    return {
        'header_frame_id': frame_id,
        'header_stamp': stamp,
        'child_frame_id': child_frame_id,
        'pose_position': (x, y, 0.0),
        'twist_linear': (linear_vx, linear_vy, 0.0),
        'twist_angular': (0.0, 0.0, angular_vz),
    }


def build_quaternion_from_yaw(theta):
    """
    Получить кватернион из угла yaw (только вращение вокруг Z).

    :return: (x, y, z, w)
    """
    return (0.0, 0.0, math.sin(theta / 2), math.cos(theta / 2))


def build_tf_data(x, y, theta, frame_id, child_frame_id, stamp):
    """
    Построить данные для TF transform.

    :return: dict с полями для TransformStamped
    """
    quat = build_quaternion_from_yaw(theta)
    return {
        'header_frame_id': frame_id,
        'header_stamp': stamp,
        'child_frame_id': child_frame_id,
        'translation': (x, y, 0.0),
        'rotation': quat,
    }


def build_marker_data(foot_positions, frame_id, stamp, marker_scale=0.05):
    """
    Построить данные для маркеров лап.

    :param foot_positions: Список из 4 кортежей (x, y, z)
    :param frame_id: Frame ID для маркеров
    :param stamp: Время
    :param marker_scale: Размер маркера
    :return: Список словарей с данными маркеров
    """
    markers = []
    for i, pos in enumerate(foot_positions):
        markers.append({
            'id': i,
            'position': (pos[0], pos[1], pos[2]),
            'scale': (marker_scale, marker_scale, marker_scale),
            'color': (1.0, 0.0, 0.0, 1.0),
            'frame_id': frame_id,
            'stamp': stamp,
        })
    return markers
