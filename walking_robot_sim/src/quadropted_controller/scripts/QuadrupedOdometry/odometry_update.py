#!/usr/bin/env python3
"""
Обновление одометрии — чистая логика (без ROS зависимостей).
Вынесено из QuadrupedOdometryNode.py при декомпозиции.
"""

import math


def normalize_angle(angle):
    """Нормализовать угол до [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def update_odometry(state, dt, contact_count_coeff=0.65):
    """
    Обновить позицию одометрии на основе контактов лап или скоростей.

    Использует state.theta (текущий угол из IMU) для поворота смещений.
    НЕ изменяет state.theta — это делается отдельно в imu_callback.

    :param state: OdometryState
    :param dt: Время с последнего обновления (секунды)
    :param contact_count_coeff: Коэффициент вклада каждой лапы (по умолч. 0.65)
    :return: None (состояние обновляется inplace)
    """
    if dt <= 0.0:
        return

    delta_x_total, delta_y_total = 0.0, 0.0
    contact_count = 0

    for i in range(4):
        if state.foot_contacts[i]:
            foot_rel_x = state.foot_positions[i][0]
            foot_rel_y = state.foot_positions[i][1]

            if state.prev_foot_positions[i] is not None:
                delta_x = foot_rel_x - state.prev_foot_positions[i][0]
                delta_y = foot_rel_y - state.prev_foot_positions[i][1]

                delta_x_total += delta_x
                delta_y_total += -delta_y
                contact_count += contact_count_coeff

            state.prev_foot_positions[i] = (foot_rel_x, foot_rel_y)

    if contact_count > 0:
        delta_x = delta_x_total / contact_count
        delta_y = delta_y_total / contact_count

        state.delta_x_queue.append(delta_x)
        state.delta_y_queue.append(delta_y)

        avg_delta_x = sum(state.delta_x_queue) / len(state.delta_x_queue)
        avg_delta_y = sum(state.delta_y_queue) / len(state.delta_y_queue)

        state.x += (avg_delta_x * math.cos(state.theta) - avg_delta_y * math.sin(state.theta))
        state.y += (avg_delta_x * math.sin(state.theta) + avg_delta_y * math.cos(state.theta))
    else:
        # Обновление на основе команд скорости
        delta_x = state.linear_velocity_x * dt
        delta_y = state.linear_velocity_y * dt

        state.delta_x_queue.append(delta_x)
        state.delta_y_queue.append(delta_y)

        avg_delta_x = sum(state.delta_x_queue) / len(state.delta_x_queue)
        avg_delta_y = sum(state.delta_y_queue) / len(state.delta_y_queue)

        state.x += (avg_delta_x * math.cos(state.theta) - avg_delta_y * math.sin(state.theta))
        state.y += (avg_delta_x * math.sin(state.theta) + avg_delta_y * math.cos(state.theta))
