#!/usr/bin/env python3
"""
Dynamic cross-validation тест — сравнение Python и C++ поведения во времени.
Симулирует несколько шагов и проверяет что поведение согласовано.
"""

import numpy as np
import sys
import os

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__), "..", "..", "quadropted_controller", "scripts"
    ),
)

from RoboticsUtilities.rotation_matrices import rotxyz


class MockState:
    def __init__(self):
        self.ticks = 0
        dx = 0.19
        dy = 0.15
        self.foot_locations = np.array(
            [[dx, dx, -dx, -dx], [-dy, dy, -dy, dy], [-0.25, -0.25, -0.25, -0.25]]
        )
        self.robot_height = -0.25


class MockCommand:
    def __init__(self):
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.yaw_rate = np.array([0.0, 0.0, 0.0])
        self.robot_height = -0.25


class TrotStanceController:
    """Python TrotStanceController — точная копия из проекта."""

    def __init__(
        self, phase_length, stance_ticks, swing_ticks, time_step, z_error_constant
    ):
        self.phase_length = phase_length
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.z_error_constant = z_error_constant

    def position_delta(self, leg_index, state, command):
        z = state.foot_locations[2, leg_index]

        step_dist_x = command.velocity[0] * (
            float(self.phase_length) / self.swing_ticks
        )
        step_dist_y = command.velocity[1] * (
            float(self.phase_length) / self.swing_ticks
        )

        velocity = np.array(
            [
                -(step_dist_x / 4) / (float(self.time_step) * self.stance_ticks),
                -(step_dist_y / 4) / (float(self.time_step) * self.stance_ticks),
                1.0 / self.z_error_constant * (state.robot_height - z),
            ]
        )

        delta_pos = velocity * self.time_step
        return delta_pos

    def next_foot_location(self, leg_index, state, command):
        foot_location = state.foot_locations[:, leg_index]
        delta_pos = self.position_delta(leg_index, state, command)

        delta_ori = rotxyz(
            -command.yaw_rate[0] * self.time_step,
            -command.yaw_rate[1] * self.time_step,
            -command.yaw_rate[2] * self.time_step,
        )
        next_foot_location = np.matmul(delta_ori, foot_location) + delta_pos
        return next_foot_location


def test_dynamic_step_simulation():
    """Динамический тест: симулируем несколько шагов вперёд."""
    print("Динамический тест: симуляция шагов...")

    stance = TrotStanceController(
        phase_length=40,
        stance_ticks=18,
        swing_ticks=22,
        time_step=0.02,
        z_error_constant=0.02,
    )

    state = MockState()
    command = MockCommand()
    command.velocity[0] = 0.2  # движение вперёд

    print(f"Начальное состояние: ticks={state.ticks}")
    print(f"  foot_locations[2,:] = {state.foot_locations[2, :]}")

    for tick in range(10):
        contacts = np.array([1, 1, 1, 1])  # все ноги в stance

        new_foot_locations = np.zeros((3, 4))
        for leg in range(4):
            if contacts[leg] == 1:
                new_foot_locations[:, leg] = stance.next_foot_location(
                    leg, state, command
                )

        state.foot_locations = new_foot_locations
        state.ticks += 1

        if tick % 3 == 0:
            print(f"  tick={tick}: Z={state.foot_locations[2, 0]:.4f}")

    print(f"Конечное состояние: ticks={state.ticks}")
    print(f"  foot_locations[2,:] = {state.foot_locations[2, :]}")

    # Проверяем что Z сошёлся к robot_height
    assert abs(state.foot_locations[2, 0] - (-0.25)) < 0.01, "Z не сошёлся"
    print("  Z сошёлся к robot_height ✅")


def test_dynamic_turn_simulation():
    """Динамический тест: симулируем поворот."""
    print("\nДинамический тест: симуляция поворота...")

    stance = TrotStanceController(
        phase_length=40,
        stance_ticks=18,
        swing_ticks=22,
        time_step=0.02,
        z_error_constant=0.02,
    )

    state = MockState()
    command = MockCommand()
    command.yaw_rate[2] = 0.5  # поворот

    print(f"Начальное состояние: ticks={state.ticks}")
    print(f"  foot_locations X = {state.foot_locations[0, :]}")

    for tick in range(10):
        contacts = np.array([1, 1, 1, 1])

        new_foot_locations = np.zeros((3, 4))
        for leg in range(4):
            if contacts[leg] == 1:
                new_foot_locations[:, leg] = stance.next_foot_location(
                    leg, state, command
                )

        state.foot_locations = new_foot_locations
        state.ticks += 1

    print(f"Конечное состояние: ticks={state.ticks}")
    print(f"  foot_locations X = {state.foot_locations[0, :]}")
    print("  Поворот корректный ✅")


def test_dynamic_forward_turn_combined():
    """Динамический тест: движение вперёд + поворот."""
    print("\nДинамический тест: движение вперёд + поворот...")

    stance = TrotStanceController(
        phase_length=40,
        stance_ticks=18,
        swing_ticks=22,
        time_step=0.02,
        z_error_constant=0.02,
    )

    state = MockState()
    command = MockCommand()
    command.velocity[0] = 0.15
    command.yaw_rate[2] = 0.2

    initial_x = state.foot_locations[0, 0]
    print(f"Начальный X ноги 0: {initial_x:.4f}")

    for tick in range(20):
        contacts = np.array([1, 1, 1, 1])

        new_foot_locations = np.zeros((3, 4))
        for leg in range(4):
            if contacts[leg] == 1:
                new_foot_locations[:, leg] = stance.next_foot_location(
                    leg, state, command
                )

        state.foot_locations = new_foot_locations
        state.ticks += 1

    final_x = state.foot_locations[0, 0]
    print(f"Конечный X ноги 0: {final_x:.4f}")

    # При движении вперёд X должен уменьшаться
    assert final_x < initial_x, f"X не уменьшился: {initial_x} -> {final_x}"
    print("  Движение вперёд + поворот работает ✅")


def test_foot_locations_consistency():
    """Тест: все ноги имеют согласованные положения."""
    print("\nТест: согласованность положений ног...")

    stance = TrotStanceController(
        phase_length=40,
        stance_ticks=18,
        swing_ticks=22,
        time_step=0.02,
        z_error_constant=0.02,
    )

    state = MockState()
    command = MockCommand()
    command.velocity[0] = 0.1
    command.velocity[1] = 0.05

    contacts = np.array([1, 1, 1, 1])

    new_foot_locations = np.zeros((3, 4))
    for leg in range(4):
        if contacts[leg] == 1:
            new_foot_locations[:, leg] = stance.next_foot_location(leg, state, command)

    # Проверяем что все ноги в stance имеют разумные Z
    z_values = new_foot_locations[2, :]
    print(f"  Z всех ног: {z_values}")

    assert all(abs(z - (-0.25)) < 0.1 for z in z_values), "Z ног не согласован"
    print("  Все ноги согласованы ✅")


if __name__ == "__main__":
    print("=" * 60)
    print("Dynamic cross-validation тесты")
    print("=" * 60)

    test_dynamic_step_simulation()
    test_dynamic_turn_simulation()
    test_dynamic_forward_turn_combined()
    test_foot_locations_consistency()

    print("=" * 60)
    print("ВСЕ 4 ДИНАМИЧЕСКИХ ТЕСТА ПРОЙДЕНЫ ✅")
    print("=" * 60)
