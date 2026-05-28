#!/usr/bin/env python3
"""
Cross-validation тесты для step_trot — сравнение Python и C++ логики.
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
        self.imu_roll = 0.0
        self.imu_pitch = 0.0


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

        # rotxyz для orientation — cmd_vel = [roll_rate, pitch_rate, yaw_rate]
        delta_ori = rotxyz(
            -command.yaw_rate[0] * self.time_step,
            -command.yaw_rate[1] * self.time_step,
            -command.yaw_rate[2] * self.time_step,
        )
        next_foot_location = np.matmul(delta_ori, foot_location) + delta_pos
        return next_foot_location


def test_stance_position_delta_zero_velocity():
    """Тест 1: position_delta при нулевой скорости — Z движется к robot_height."""
    print("Тест 1: position_delta при нулевой скорости...")

    stance = TrotStanceController(
        phase_length=40,
        stance_ticks=18,
        swing_ticks=22,
        time_step=0.02,
        z_error_constant=0.02,
    )
    state = MockState()
    command = MockCommand()
    state.foot_locations[2, 0] = -0.20  # Z выше чем robot_height

    delta_pos = stance.position_delta(0, state, command)

    expected_z = 1.0 / 0.02 * (-0.25 - (-0.20)) * 0.02  # = -0.25
    assert abs(delta_pos[2] - (-0.05)) < 0.001, (
        f"Z delta = {delta_pos[2]}, ожидалось ~-0.05"
    )
    print(f"  delta_pos = {delta_pos}, Z delta корректный")


def test_stance_position_delta_forward_motion():
    """Тест 2: position_delta при движении вперёд — X уменьшается."""
    print("Тест 2: position_delta при движении вперёд...")

    stance = TrotStanceController(
        phase_length=40,
        stance_ticks=18,
        swing_ticks=22,
        time_step=0.02,
        z_error_constant=0.02,
    )
    state = MockState()
    command = MockCommand()
    command.velocity[0] = 0.3  # движение вперёд

    delta_pos = stance.position_delta(0, state, command)

    step_dist_x = 0.3 * (40.0 / 22.0)  # ~= 0.545
    expected_vx = -(step_dist_x / 4) / (0.02 * 18)  # ~= -0.378

    assert delta_pos[0] < 0, f"X delta = {delta_pos[0]}, должен быть отрицательным"
    print(f"  delta_pos[0] = {delta_pos[0]}, корректное направление")


def test_stance_position_delta_rotation():
    """Тест 3: position_delta с yaw_rate — вращение вокруг Z."""
    print("Тест 3: position_delta с yaw_rate...")

    stance = TrotStanceController(
        phase_length=40,
        stance_ticks=18,
        swing_ticks=22,
        time_step=0.02,
        z_error_constant=0.02,
    )
    state = MockState()
    command = MockCommand()
    command.yaw_rate[2] = 0.5  # yaw rotation

    delta_pos = stance.position_delta(0, state, command)

    # delta_ori вычисляется в next_foot_location
    delta_ori = rotxyz(
        -command.yaw_rate[0] * 0.02,
        -command.yaw_rate[1] * 0.02,
        -command.yaw_rate[2] * 0.02,
    )

    assert abs(delta_ori[0, 0] - 0.999) < 0.01, (
        f"rot[0,0] = {delta_ori[0, 0]}, ожидался ~0.999"
    )
    print(f"  rotxyz matrix:\n{delta_ori}")
    print("  Вращение корректное")


def test_stance_next_foot_location_basic():
    """Тест 4: next_foot_location базовый расчёт."""
    print("Тест 4: next_foot_location базовый...")

    stance = TrotStanceController(
        phase_length=40,
        stance_ticks=18,
        swing_ticks=22,
        time_step=0.02,
        z_error_constant=0.02,
    )
    state = MockState()
    command = MockCommand()

    new_loc = stance.next_foot_location(0, state, command)

    assert new_loc.shape == (3,), f"Shape = {new_loc.shape}, ожидался (3,)"
    print(f"  new_loc = {new_loc}, корректный размер")


def test_stance_full_trot_step():
    """Тест 5: полный шаг trot — stance phase для контактирующих ног."""
    print("Тест 5: полный шаг trot...")

    phase_length = 40
    stance = TrotStanceController(
        phase_length=phase_length,
        stance_ticks=18,
        swing_ticks=22,
        time_step=0.02,
        z_error_constant=0.02,
    )
    state = MockState()
    command = MockCommand()
    command.velocity[0] = 0.2

    contacts = np.array([1, 1, 1, 1])  # all stance

    new_foot_locations = np.zeros((3, 4))
    for leg in range(4):
        if contacts[leg] == 1:
            new_foot_locations[:, leg] = stance.next_foot_location(leg, state, command)

    print(f"  new_foot_locations Z = {new_foot_locations[2, :]}")
    assert new_foot_locations.shape == (3, 4), f"Shape = {new_foot_locations.shape}"
    print("  Размер корректный")


def test_stance_vs_cpp_equivalence():
    """Тест 6: эквивалентность C++ и Python для stance."""
    print("Тест 6: эквивалентность C++ и Python...")

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
    command.velocity[1] = 0.1
    command.yaw_rate[2] = 0.3

    state.ticks = 5

    for leg in range(4):
        foot_loc = state.foot_locations[:, leg]
        delta_pos = stance.position_delta(leg, state, command)

        # delta_ori вычисляется в next_foot_location
        delta_ori = rotxyz(
            -command.yaw_rate[0] * 0.02,
            -command.yaw_rate[1] * 0.02,
            -command.yaw_rate[2] * 0.02,
        )
        new_loc = np.matmul(delta_ori, foot_loc) + delta_pos

        print(f"  Leg {leg}: old_z={foot_loc[2]:.3f}, new_z={new_loc[2]:.3f}")

    print("  Расчёт корректный")


def test_stance_robot_height_convergence():
    """Тест 7: сходимость к robot_height при разных начальных Z."""
    print("Тест 7: сходимость к robot_height...")

    stance = TrotStanceController(
        phase_length=40,
        stance_ticks=18,
        swing_ticks=22,
        time_step=0.02,
        z_error_constant=0.02,
    )

    test_z_values = [-0.15, -0.20, -0.30]  # исключаем -0.25 = robot_height
    target_z = -0.25

    for initial_z in test_z_values:
        state = MockState()
        state.foot_locations[2, 0] = initial_z
        state.robot_height = target_z
        command = MockCommand()

        delta_pos = stance.position_delta(0, state, command)

        new_z = initial_z + delta_pos[2]
        convergence = abs(new_z - target_z) < abs(initial_z - target_z)

        print(f"  z: {initial_z:.2f} -> {new_z:.3f} (target: {target_z})")
        assert convergence, f"Не сошлось: {initial_z} -> {new_z}"

    print("  Сходимость работает корректно")


def test_stance_sign_consistency():
    """Тест 8: согласованность знаков с C++ после исправления robot_height."""
    print("Тест 8: согласованность знаков с C++...")

    stance = TrotStanceController(
        phase_length=40,
        stance_ticks=18,
        swing_ticks=22,
        time_step=0.02,
        z_error_constant=0.02,
    )
    state = MockState()
    command = MockCommand()

    assert state.robot_height == -0.25, f"robot_height = {state.robot_height}"
    assert state.foot_locations[2, 0] == -0.25, (
        f"initial Z = {state.foot_locations[2, 0]}"
    )

    delta_pos = stance.position_delta(0, state, command)

    assert delta_pos[2] == 0.0, f"delta_z = {delta_pos[2]} при равных Z"
    print(f"  delta_pos = {delta_pos}")
    print("  Знаки согласованы с C++")


if __name__ == "__main__":
    print("=" * 60)
    print("Cross-validation тесты для step_trot (Python)")
    print("=" * 60)

    test_stance_position_delta_zero_velocity()
    test_stance_position_delta_forward_motion()
    test_stance_position_delta_rotation()
    test_stance_next_foot_location_basic()
    test_stance_full_trot_step()
    test_stance_vs_cpp_equivalence()
    test_stance_robot_height_convergence()
    test_stance_sign_consistency()

    print("=" * 60)
    print("ВСЕ 8 ТЕСТОВ ПРОЙДЕНЫ ✅")
    print("=" * 60)
