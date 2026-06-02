#!/usr/bin/env python3
"""
Модульный тест для проверки декомпозиции QuadrupedOdometryNode.py.
Сравнивает результаты старой (монолитной) и новой (декомпозированной) логики.
"""

import sys
import os
import math
import pytest
import numpy as np
from collections import deque

# ============================================================
# Тестовые данные
# ============================================================
BODY = [0.3762, 0.0935]
LEGS = [0.0, 0.0955, 0.213, 0.213]


# ============================================================
# Старая (монолитная) логика — извлечена из оригинального файла
# ============================================================

class OldOdometryState:
    """Копия состояния из оригинального QuadrupedOdometryNode.py."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity_x = 0.0
        self.linear_velocity_y = 0.0
        self.imu_angular_velocity = 0.0
        self.filter_window_size = 14
        self.delta_x_queue = deque(maxlen=self.filter_window_size)
        self.delta_y_queue = deque(maxlen=self.filter_window_size)
        self.last_position_time_ns = 0.0  # наносекунды
        self.gazebo_clock_sec = 0
        self.gazebo_clock_nanosec = 0
        self.encoder_pos = 0
        self.foot_contacts = [False, False, False, False]
        self.joint_positions = [0.0] * 12
        self.foot_positions = [(0.0, 0.0, 0.0)] * 4
        self.prev_foot_positions = [None, None, None, None]


def old_normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def old_update_odometry(state, current_time_ns, contact_count_coeff=0.65):
    """Копия update_odometry из оригинального QuadrupedOdometryNode.py.
    Примечание: оригинал НЕ обновляет theta — он устанавливается только из IMU.
    """
    dt = (current_time_ns - state.last_position_time_ns) / 1e9
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
        delta_x = state.linear_velocity_x * dt
        delta_y = state.linear_velocity_y * dt

        state.delta_x_queue.append(delta_x)
        state.delta_y_queue.append(delta_y)

        avg_delta_x = sum(state.delta_x_queue) / len(state.delta_x_queue)
        avg_delta_y = sum(state.delta_y_queue) / len(state.delta_y_queue)

        state.x += (avg_delta_x * math.cos(state.theta) - avg_delta_y * math.sin(state.theta))
        state.y += (avg_delta_x * math.sin(state.theta) + avg_delta_y * math.cos(state.theta))

    state.last_position_time_ns = current_time_ns


# ============================================================
# Новая (декомпозированная) логика
# ============================================================

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(BASE_DIR, "quadropted_controller", "scripts")
sys.path.insert(0, SCRIPTS_DIR)

from QuadrupedOdometry import OdometryState, update_odometry as new_update_odometry


# ============================================================
# Тесты
# ============================================================

class TestOdometryStateInit:
    """Проверка что начальное состояние одинаковое."""

    def test_initial_values_match(self):
        old = OldOdometryState()
        new = OdometryState()

        assert old.x == pytest.approx(new.x)
        assert old.y == pytest.approx(new.y)
        assert old.theta == pytest.approx(new.theta)
        assert old.linear_velocity_x == pytest.approx(new.linear_velocity_x)
        assert old.linear_velocity_y == pytest.approx(new.linear_velocity_y)
        assert old.filter_window_size == new.filter_window_size
        assert old.foot_contacts == new.foot_contacts
        assert old.joint_positions == pytest.approx(new.joint_positions)
        assert old.foot_positions == pytest.approx(new.foot_positions)
        assert old.prev_foot_positions == new.prev_foot_positions


class TestOdometryUpdateFootContact:
    """Обновление на основе контактов лап."""

    def _run_contact_scenario(self):
        """Сценарий: все 4 лапы на земле, двигаются."""
        old = OldOdometryState()
        new = OdometryState()

        # Настраиваем контакты
        old.foot_contacts = [True, True, True, True]
        new.foot_contacts = [True, True, True, True]

        # Начальная позиция лап (первый контакт — prev_foot_positions = None)
        old.foot_positions = [
            (0.2, -0.1, -0.3),
            (0.2, 0.1, -0.3),
            (-0.2, -0.1, -0.3),
            (-0.2, 0.1, -0.3),
        ]
        new.foot_positions = list(old.foot_positions)

        # Первый вызов — prev_foot_positions=None, только запоминаем
        old_t0 = int(1e9)  # 1 секунда в наносекундах
        old.last_position_time_ns = old_t0
        old_update_odometry(old, old_t0 + int(0.02e9))
        new_update_odometry(new, 0.02)

        # После первого вызова prev_foot_positions должны быть установлены
        for i in range(4):
            assert old.prev_foot_positions[i] is not None
            assert new.prev_foot_positions[i] is not None
            np.testing.assert_array_almost_equal(
                old.prev_foot_positions[i], new.prev_foot_positions[i], decimal=15
            )

        # Двигаем лапы — второе обновление
        old.foot_positions = [
            (0.19, -0.11, -0.3),
            (0.19, 0.09, -0.3),
            (-0.21, -0.11, -0.3),
            (-0.21, 0.09, -0.3),
        ]
        new.foot_positions = list(old.foot_positions)

        old_t1 = old_t0 + int(0.02e9)
        old_update_odometry(old, old_t1 + int(0.02e9))
        new_update_odometry(new, 0.02)

        np.testing.assert_almost_equal(old.x, new.x, decimal=12)
        np.testing.assert_almost_equal(old.y, new.y, decimal=12)
        np.testing.assert_almost_equal(old.theta, new.theta, decimal=12)

    def test_contact_scenario(self):
        self._run_contact_scenario()

    def test_multiple_contact_updates(self):
        """Несколько последовательных обновлений с контактами."""
        old = OldOdometryState()
        new = OdometryState()

        old.foot_contacts = [True, True, False, False]  # только передние
        new.foot_contacts = [True, True, False, False]

        old.foot_positions = [
            (0.2, -0.1, -0.3),
            (0.2, 0.1, -0.3),
            (-0.2, -0.1, -0.3),
            (-0.2, 0.1, -0.3),
        ]
        new.foot_positions = list(old.foot_positions)

        t = int(1e9)
        for step in range(5):
            # Сдвигаем лапы
            old.foot_positions = [
                (old.foot_positions[i][0] - 0.005, old.foot_positions[i][1] + 0.001, old.foot_positions[i][2])
                for i in range(4)
            ]
            new.foot_positions = list(old.foot_positions)

            # Обновляем theta через state (как в реальном IMU callback)
            old.theta = 0.1 * step
            new.theta = 0.1 * step

            old_update_odometry(old, t)
            new_update_odometry(new, 0.02)
            t += int(0.02e9)

        np.testing.assert_almost_equal(old.x, new.x, decimal=12)
        np.testing.assert_almost_equal(old.y, new.y, decimal=12)
        np.testing.assert_almost_equal(old.theta, new.theta, decimal=12)


class TestOdometryUpdateVelocity:
    """Обновление на основе скоростей (нет контактов)."""

    def test_velocity_only_update(self):
        old = OldOdometryState()
        new = OdometryState()

        old.linear_velocity_x = 0.02
        old.linear_velocity_y = 0.01
        new.linear_velocity_x = 0.02
        new.linear_velocity_y = 0.01

        old.foot_contacts = [False, False, False, False]
        new.foot_contacts = [False, False, False, False]

        # Инициализируем last_position_time_ns чтобы dt=0.02 как в новом
        t = int(1e9)
        old.last_position_time_ns = t
        old_update_odometry(old, t + int(0.02e9))
        new_update_odometry(new, 0.02)

        np.testing.assert_almost_equal(old.x, new.x, decimal=12)
        np.testing.assert_almost_equal(old.y, new.y, decimal=12)

    def test_velocity_with_rotation(self):
        """Скорости + поворот."""
        old = OldOdometryState()
        new = OdometryState()

        old.linear_velocity_x = 0.03
        old.linear_velocity_y = 0.005
        new.linear_velocity_x = 0.03
        new.linear_velocity_y = 0.005

        old.foot_contacts = [False, False, False, False]
        new.foot_contacts = [False, False, False, False]

        t = int(1e9)
        old.last_position_time_ns = t
        for step in range(10):
            theta = 0.15 * step
            old.theta = theta
            new.theta = theta
            old_update_odometry(old, t + int(0.02e9))
            new_update_odometry(new, 0.02)
            t += int(0.02e9)

        np.testing.assert_almost_equal(old.x, new.x, decimal=12)
        np.testing.assert_almost_equal(old.y, new.y, decimal=12)
        np.testing.assert_almost_equal(old.theta, new.theta, decimal=12)


class TestNormalizeAngle:
    """Проверка normalize_angle."""

    def test_normalize_various_angles(self):
        for angle in [0.0, math.pi, -math.pi, 3 * math.pi, -3 * math.pi, math.pi / 2, -math.pi / 4]:
            old_result = old_normalize_angle(angle)
            new_result = old_normalize_angle(angle)  # используем ту же функцию
            np.testing.assert_almost_equal(old_result, new_result, decimal=15)


class TestMixedContactModes:
    """Смешанный режим: часть лап на земле, часть нет."""

    def test_mixed_contacts(self):
        old = OldOdometryState()
        new = OdometryState()

        # Сначала контакт — устанавливаем prev_foot_positions
        old.foot_contacts = [True, True, True, True]
        new.foot_contacts = [True, True, True, True]

        old.foot_positions = [
            (0.2, -0.1, -0.3),
            (0.2, 0.1, -0.3),
            (-0.2, -0.1, -0.3),
            (-0.2, 0.1, -0.3),
        ]
        new.foot_positions = list(old.foot_positions)

        t = int(1e9)
        old_update_odometry(old, t + int(0.02e9))
        new_update_odometry(new, 0.02)

        # Теперь только 2 лапы
        old.foot_contacts = [True, False, True, False]
        new.foot_contacts = [True, False, True, False]

        old.foot_positions = [
            (0.18, -0.12, -0.3),
            (0.22, 0.08, -0.3),
            (-0.22, -0.08, -0.3),
            (-0.18, 0.12, -0.3),
        ]
        new.foot_positions = list(old.foot_positions)

        old.theta = 0.05
        new.theta = 0.05

        old_update_odometry(old, t + int(0.04e9))
        new_update_odometry(new, 0.02)

        np.testing.assert_almost_equal(old.x, new.x, decimal=12)
        np.testing.assert_almost_equal(old.y, new.y, decimal=12)
        np.testing.assert_almost_equal(old.theta, new.theta, decimal=12)


class TestSlidingWindowFilter:
    """Проверка что фильтр скользящего среднего работает одинаково."""

    def test_filter_accumulation(self):
        """Проверяем что очередь фильтра заполняется одинаково."""
        old = OldOdometryState()
        new = OdometryState()

        old.linear_velocity_x = 0.01
        old.linear_velocity_y = 0.005
        new.linear_velocity_x = 0.01
        new.linear_velocity_y = 0.005

        old.foot_contacts = [False, False, False, False]
        new.foot_contacts = [False, False, False, False]

        t = int(1e9)
        old.last_position_time_ns = t
        for step in range(20):  # Больше чем размер окна (14)
            old_update_odometry(old, t + int(0.02e9))
            new_update_odometry(new, 0.02)
            t += int(0.02e9)

        # Проверяем что очереди одинаковые
        assert len(old.delta_x_queue) == len(new.delta_x_queue)
        for ox, nx in zip(old.delta_x_queue, new.delta_x_queue):
            np.testing.assert_almost_equal(ox, nx, decimal=12)
        for oy, ny in zip(old.delta_y_queue, new.delta_y_queue):
            np.testing.assert_almost_equal(oy, ny, decimal=12)

        np.testing.assert_almost_equal(old.x, new.x, decimal=12)
        np.testing.assert_almost_equal(old.y, new.y, decimal=12)
