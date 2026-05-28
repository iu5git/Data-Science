#!/usr/bin/env python3
"""
Модульный тест для проверки декомпозиции.
Сравнивает результаты исходных и декомпозированных модулей.
Никакой подстройки под ошибки — только строгое сравнение.
"""

import sys
import os
import pytest
import numpy as np

# Добавляем пути к исходному и новому коду
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(BASE_DIR, "quadropted_controller", "scripts")
DECOMP_DIR = os.path.join(SCRIPTS_DIR, "decomposition")

# Исходный код
sys.path.insert(0, SCRIPTS_DIR)

# Декомпозированный код
sys.path.insert(0, DECOMP_DIR)

# ============================================================
# Тестовые данные
# ============================================================
BODY = [0.3762, 0.0935]
LEGS = [0.0, 0.0955, 0.213, 0.213]

# Реалистичные позиции ног (из стандартной стойки робота)
# Эти позиции гарантированно проходят через IK без domain error
DEFAULT_STANCE = np.array(
    [
        [0.2081, 0.2081, -0.1881, -0.1881],
        [-0.1425, 0.1425, -0.1425, 0.1425],
        [0, 0, 0, 0],
    ]
)

# Тестовые параметры тела
DX, DY, DZ = 0.0, 0.0, 0.0
ROLL, PITCH, YAW = 0.0, 0.0, 0.0


# ============================================================
# Transformations
# ============================================================
class TestTransformations:
    def test_rotx(self):
        from RoboticsUtilities.Transformations import rotx as old_rotx
        from decomposition.transformations.rotation_matrices import rotx as new_rotx

        for angle in [0.0, 0.5, -0.3, np.pi / 4, np.pi / 2]:
            old = old_rotx(angle)
            new = new_rotx(angle)
            np.testing.assert_array_almost_equal(old, new, decimal=15)

    def test_roty(self):
        from RoboticsUtilities.Transformations import roty as old_roty
        from decomposition.transformations.rotation_matrices import roty as new_roty

        for angle in [0.0, 0.5, -0.3, np.pi / 4, np.pi / 2]:
            old = old_roty(angle)
            new = new_roty(angle)
            np.testing.assert_array_almost_equal(old, new, decimal=15)

    def test_rotz(self):
        from RoboticsUtilities.Transformations import rotz as old_rotz
        from decomposition.transformations.rotation_matrices import rotz as new_rotz

        for angle in [0.0, 0.5, -0.3, np.pi / 4, np.pi / 2]:
            old = old_rotz(angle)
            new = new_rotz(angle)
            np.testing.assert_array_almost_equal(old, new, decimal=15)

    def test_rotxyz(self):
        from RoboticsUtilities.Transformations import rotxyz as old_rotxyz
        from decomposition.transformations.rotation_matrices import rotxyz as new_rotxyz

        angles = [(0.1, 0.2, 0.3), (-0.5, 0.0, 0.1), (0.0, 0.0, 0.0)]
        for a, b, g in angles:
            old = old_rotxyz(a, b, g)
            new = new_rotxyz(a, b, g)
            np.testing.assert_array_almost_equal(old, new, decimal=15)

    def test_homog_transxyz(self):
        from RoboticsUtilities.Transformations import homog_transxyz as old_trans
        from decomposition.transformations.homogeneous_transforms import (
            homog_transxyz as new_trans,
        )

        for dx, dy, dz in [(0.1, 0.2, 0.3), (-1.0, 0.0, 0.5), (0.0, 0.0, 0.0)]:
            old = old_trans(dx, dy, dz)
            new = new_trans(dx, dy, dz)
            np.testing.assert_array_almost_equal(old, new, decimal=15)

    def test_homog_transform(self):
        from RoboticsUtilities.Transformations import homog_transform as old_ht
        from decomposition.transformations.homogeneous_transforms import (
            homog_transform as new_ht,
        )

        params = [
            (0.1, 0.2, 0.3, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, np.pi / 4, np.pi / 3, np.pi / 6),
            (0.3762 / 2, 0.0935 / 2, 0.0, np.pi / 2, -np.pi / 2, 0),
        ]
        for dx, dy, dz, a, b, g in params:
            old = old_ht(dx, dy, dz, a, b, g)
            new = new_ht(dx, dy, dz, a, b, g)
            np.testing.assert_array_almost_equal(old, new, decimal=15)

    def test_homog_transform_inverse(self):
        from RoboticsUtilities.Transformations import homog_transform_inverse as old_inv
        from decomposition.transformations.homogeneous_transforms import (
            homog_transform_inverse as new_inv,
        )

        params = [
            (0.1, 0.2, 0.3, 0.1, 0.2, 0.3),
            (0.0, 0.0, 0.0, np.pi / 4, np.pi / 3, np.pi / 6),
            (0.3762 / 2, -0.0935 / 2, 0, np.pi / 2, -np.pi / 2, 0),
        ]
        for dx, dy, dz, a, b, g in params:
            from RoboticsUtilities.Transformations import homog_transform as ht

            matrix = ht(dx, dy, dz, a, b, g)
            old = old_inv(matrix.copy())
            new = new_inv(matrix.copy())
            np.testing.assert_array_almost_equal(old, new, decimal=15)


# ============================================================
# Forward Kinematics
# ============================================================
class TestForwardKinematics:
    def test_forward_kinematics_per_leg(self):
        from ForwardKinematics.robot_FK import ForwardKinematics
        from decomposition.forward_kinematics.forward_kinematics import (
            ForwardKinematics as NewFK,
        )

        old_fk = ForwardKinematics(BODY, LEGS)
        new_fk = NewFK(BODY, LEGS)

        # Маленькие углы — гарантированно проходят
        small_angles = [0.0, 0.1, -0.1, 0.0, 0.1, -0.1, 0.0, 0.1, -0.1, 0.0, 0.1, -0.1]

        for leg_idx in range(4):
            theta_hip = small_angles[leg_idx * 3]
            theta_thigh = small_angles[leg_idx * 3 + 1]
            theta_calf = small_angles[leg_idx * 3 + 2]

            old_pos = old_fk.forward_kinematics_per_leg(
                theta_hip, theta_thigh, theta_calf, leg_idx
            )
            new_pos = new_fk.forward_kinematics_per_leg(
                theta_hip, theta_thigh, theta_calf, leg_idx
            )
            np.testing.assert_array_almost_equal(old_pos, new_pos, decimal=15)

    def test_forward_kinematics_all_legs(self):
        from ForwardKinematics.robot_FK import ForwardKinematics
        from decomposition.forward_kinematics.forward_kinematics import (
            ForwardKinematics as NewFK,
        )

        old_fk = ForwardKinematics(BODY, LEGS)
        new_fk = NewFK(BODY, LEGS)

        small_angles = [0.0, 0.1, -0.1, 0.0, 0.1, -0.1, 0.0, 0.1, -0.1, 0.0, 0.1, -0.1]

        old_positions = old_fk.forward_kinematics_all_legs(small_angles)
        new_positions = new_fk.forward_kinematics_all_legs(small_angles)

        for old_pos, new_pos in zip(old_positions, new_positions):
            np.testing.assert_array_almost_equal(old_pos, new_pos, decimal=15)


# ============================================================
# Inverse Kinematics
# ============================================================
class TestInverseKinematics:
    def test_get_local_positions(self):
        from InverseKinematics.robot_IK import InverseKinematics
        from decomposition.inverse_kinematics.inverse_kinematics import (
            InverseKinematics as NewIK,
        )

        old_ik = InverseKinematics(BODY, LEGS)
        new_ik = NewIK(BODY, LEGS)

        old_positions = old_ik.get_local_positions(
            DEFAULT_STANCE, DX, DY, DZ, ROLL, PITCH, YAW
        )
        new_positions = new_ik.get_local_positions(
            DEFAULT_STANCE, DX, DY, DZ, ROLL, PITCH, YAW
        )

        for old_pos, new_pos in zip(old_positions, new_positions):
            np.testing.assert_array_almost_equal(old_pos, new_pos, decimal=13)

    def test_inverse_kinematics(self):
        from InverseKinematics.robot_IK import InverseKinematics
        from decomposition.inverse_kinematics.inverse_kinematics import (
            InverseKinematics as NewIK,
        )

        old_ik = InverseKinematics(BODY, LEGS)
        new_ik = NewIK(BODY, LEGS)

        old_angles = old_ik.inverse_kinematics(
            DEFAULT_STANCE, DX, DY, DZ, ROLL, PITCH, YAW
        )
        new_angles = new_ik.inverse_kinematics(
            DEFAULT_STANCE, DX, DY, DZ, ROLL, PITCH, YAW
        )

        for old_a, new_a in zip(old_angles, new_angles):
            np.testing.assert_almost_equal(old_a, new_a, decimal=13)

    def test_inverse_kinematics_multiple_configs(self):
        """Тестируем IK на нескольких валидных конфигурациях"""
        from InverseKinematics.robot_IK import InverseKinematics
        from decomposition.inverse_kinematics.inverse_kinematics import (
            InverseKinematics as NewIK,
        )

        old_ik = InverseKinematics(BODY, LEGS)
        new_ik = NewIK(BODY, LEGS)

        configs = [
            (DEFAULT_STANCE, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        ]

        for leg_pos, dx, dy, dz, roll, pitch, yaw in configs:
            old_angles = old_ik.inverse_kinematics(
                leg_pos, dx, dy, dz, roll, pitch, yaw
            )
            new_angles = new_ik.inverse_kinematics(
                leg_pos, dx, dy, dz, roll, pitch, yaw
            )

            for old_a, new_a in zip(old_angles, new_angles):
                np.testing.assert_almost_equal(old_a, new_a, decimal=13)


# ============================================================
# Trot Gait Controller — чистые классы без ROS зависимостей
# ============================================================


# Исходный TrotSwingController (копия без ROS импортов)
class _OldTrotSwingController:
    def __init__(
        self,
        stance_ticks,
        swing_ticks,
        time_step,
        phase_length,
        z_leg_lift,
        default_stance,
    ):
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.phase_length = phase_length
        self.z_leg_lift = z_leg_lift
        self.default_stance = default_stance

    def raibert_touchdown_location(self, leg_index, command):
        from RoboticsUtilities.Transformations import rotz

        scale_factor = 1.0
        delta_pos_2d = (
            command.velocity * self.phase_length * self.time_step * scale_factor
        )
        delta_pos = np.array([delta_pos_2d[0], delta_pos_2d[1], 0])
        theta = self.stance_ticks * self.time_step * command.yaw_rate[2]
        rotation = rotz(theta)
        return np.matmul(rotation, self.default_stance[:, leg_index]) + delta_pos

    def swing_height(self, swing_phase):
        scale_factor = 1.0
        if swing_phase < 0.5:
            swing_height_ = (swing_phase / 0.5) * self.z_leg_lift * scale_factor
        else:
            swing_height_ = (
                self.z_leg_lift * (1 - (swing_phase - 0.5) / 0.5) * scale_factor
            )
        return swing_height_


# Исходный TrotStanceController (копия без ROS импортов)
class _OldTrotStanceController:
    def __init__(
        self, phase_length, stance_ticks, swing_ticks, time_step, z_error_constant
    ):
        self.phase_length = phase_length
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.z_error_constant = z_error_constant

    def position_delta(self, leg_index, state, command):
        from RoboticsUtilities.Transformations import rotxyz

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
        delta_ori = rotxyz(
            -command.yaw_rate[0] * self.time_step,
            -command.yaw_rate[1] * self.time_step,
            -command.yaw_rate[2] * self.time_step,
        )
        return (delta_pos, delta_ori)


class TestTrotGaitSwingController:
    def test_swing_height(self):
        from decomposition.trot_gait.trot_swing import TrotSwingController as NewSwing

        old_swing = _OldTrotSwingController(
            stance_ticks=2,
            swing_ticks=9,
            time_step=0.02,
            phase_length=20,
            z_leg_lift=0.14,
            default_stance=DEFAULT_STANCE,
        )
        new_swing = NewSwing(
            stance_ticks=2,
            swing_ticks=9,
            time_step=0.02,
            phase_length=20,
            z_leg_lift=0.14,
            default_stance=DEFAULT_STANCE,
        )

        for phase in [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0]:
            old_h = old_swing.swing_height(phase)
            new_h = new_swing.swing_height(phase)
            np.testing.assert_almost_equal(old_h, new_h, decimal=15)

    def test_raibert_touchdown_location(self):
        from decomposition.trot_gait.trot_swing import TrotSwingController as NewSwing

        old_swing = _OldTrotSwingController(
            stance_ticks=2,
            swing_ticks=9,
            time_step=0.02,
            phase_length=20,
            z_leg_lift=0.14,
            default_stance=DEFAULT_STANCE,
        )
        new_swing = NewSwing(
            stance_ticks=2,
            swing_ticks=9,
            time_step=0.02,
            phase_length=20,
            z_leg_lift=0.14,
            default_stance=DEFAULT_STANCE,
        )

        class MockCommand:
            def __init__(self):
                self.velocity = np.array([0.01, 0.005, 0.0])
                self.yaw_rate = np.array([0.0, 0.0, 0.1])

        command = MockCommand()
        for leg_idx in range(4):
            old_loc = old_swing.raibert_touchdown_location(leg_idx, command)
            new_loc = new_swing.raibert_touchdown_location(leg_idx, command)
            np.testing.assert_array_almost_equal(old_loc, new_loc, decimal=15)


class TestTrotGaitStanceController:
    def test_position_delta(self):
        from decomposition.trot_gait.trot_stance import (
            TrotStanceController as NewStance,
        )

        old_stance = _OldTrotStanceController(
            phase_length=20,
            stance_ticks=2,
            swing_ticks=9,
            time_step=0.02,
            z_error_constant=0.02,
        )
        new_stance = NewStance(
            phase_length=20,
            stance_ticks=2,
            swing_ticks=9,
            time_step=0.02,
            z_error_constant=0.02,
        )

        class MockState:
            def __init__(self):
                self.foot_locations = DEFAULT_STANCE.copy()
                self.robot_height = -0.25

        class MockCommand:
            def __init__(self):
                self.velocity = np.array([0.01, 0.005, 0.0])
                self.yaw_rate = np.array([0.0, 0.0, 0.1])

        state = MockState()
        command = MockCommand()

        for leg_idx in range(4):
            old_delta_pos, old_delta_ori = old_stance.position_delta(
                leg_idx, state, command
            )
            new_delta_pos, new_delta_ori = new_stance.position_delta(
                leg_idx, state, command
            )
            np.testing.assert_array_almost_equal(
                old_delta_pos, new_delta_pos, decimal=15
            )
            np.testing.assert_array_almost_equal(
                old_delta_ori, new_delta_ori, decimal=15
            )


# ============================================================
# Crawl Gait Controller — чистые классы без ROS зависимостей
# ============================================================


class _OldCrawlSwingController:
    def __init__(
        self,
        stance_ticks,
        swing_ticks,
        time_step,
        phase_length,
        z_leg_lift,
        default_stance,
        body_shift_y,
    ):
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.phase_length = phase_length
        self.z_leg_lift = z_leg_lift
        self.default_stance = default_stance
        self.body_shift_y = body_shift_y

    def raibert_touchdown_location(self, leg_index, command, shifted_left):
        from RoboticsUtilities.Transformations import rotz

        delta_pos_2d = command.velocity * self.phase_length * self.time_step
        delta_pos = np.array([delta_pos_2d[0], delta_pos_2d[1], 0])
        theta = self.stance_ticks * self.time_step * command.yaw_rate
        rotation = rotz(theta)
        shift_correction = np.array([0.0, 0.0, 0.0])
        shift_correction[1] = -self.body_shift_y if shifted_left else self.body_shift_y
        return (
            np.matmul(rotation, self.default_stance[:, leg_index])
            + delta_pos
            + shift_correction
        )

    def swing_height(self, swing_phase):
        if swing_phase < 0.5:
            return swing_phase / 0.5 * self.z_leg_lift
        else:
            return self.z_leg_lift * (1 - (swing_phase - 0.5) / 0.5)


class _OldCrawlStanceController:
    def __init__(
        self,
        phase_length,
        stance_ticks,
        swing_ticks,
        time_step,
        z_error_constant,
        body_shift_y,
    ):
        self.phase_length = phase_length
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.z_error_constant = z_error_constant
        self.body_shift_y = body_shift_y

    def position_delta(
        self, leg_index, state, command, first_cycle, move_sideways, move_left
    ):
        from RoboticsUtilities.Transformations import rotz

        z = state.foot_locations[2, leg_index]
        step_dist_x = command.velocity[0] * (
            float(self.phase_length) / self.swing_ticks
        )
        shift_factor = 1 if first_cycle else 2
        side_vel = 0.0
        if move_sideways:
            side_vel = (
                -(self.body_shift_y * shift_factor)
                / (float(self.time_step) * self.stance_ticks)
                if move_left
                else (self.body_shift_y * shift_factor)
                / (float(self.time_step) * self.stance_ticks)
            )
        velocity = np.array(
            [
                -(step_dist_x / 3) / (float(self.time_step) * self.stance_ticks),
                side_vel,
                1.0 / self.z_error_constant * (state.robot_height - z),
            ]
        )
        delta_pos = velocity * self.time_step
        delta_ori = rotz(-command.yaw_rate * self.time_step)
        return delta_pos, delta_ori


class TestCrawlGaitSwingController:
    def test_swing_height(self):
        from decomposition.crawl_gait.crawl_swing import (
            CrawlSwingController as NewSwing,
        )

        old_swing = _OldCrawlSwingController(
            stance_ticks=27,
            swing_ticks=22,
            time_step=0.02,
            phase_length=200,
            z_leg_lift=0.14,
            default_stance=DEFAULT_STANCE,
            body_shift_y=0.06,
        )
        new_swing = NewSwing(
            stance_ticks=27,
            swing_ticks=22,
            time_step=0.02,
            phase_length=200,
            z_leg_lift=0.14,
            default_stance=DEFAULT_STANCE,
            body_shift_y=0.06,
        )

        for phase in [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0]:
            old_h = old_swing.swing_height(phase)
            new_h = new_swing.swing_height(phase)
            np.testing.assert_almost_equal(old_h, new_h, decimal=15)

    def test_raibert_touchdown_location(self):
        from decomposition.crawl_gait.crawl_swing import (
            CrawlSwingController as NewSwing,
        )

        old_swing = _OldCrawlSwingController(
            stance_ticks=27,
            swing_ticks=22,
            time_step=0.02,
            phase_length=200,
            z_leg_lift=0.14,
            default_stance=DEFAULT_STANCE,
            body_shift_y=0.06,
        )
        new_swing = NewSwing(
            stance_ticks=27,
            swing_ticks=22,
            time_step=0.02,
            phase_length=200,
            z_leg_lift=0.14,
            default_stance=DEFAULT_STANCE,
            body_shift_y=0.06,
        )

        class MockCommand:
            def __init__(self):
                self.velocity = np.array([0.005, 0.0, 0.0])
                self.yaw_rate = 0.05

        command = MockCommand()
        for leg_idx in range(4):
            for shifted_left in [True, False]:
                old_loc = old_swing.raibert_touchdown_location(
                    leg_idx, command, shifted_left
                )
                new_loc = new_swing.raibert_touchdown_location(
                    leg_idx, command, shifted_left
                )
                np.testing.assert_array_almost_equal(old_loc, new_loc, decimal=15)


class TestCrawlGaitStanceController:
    def test_position_delta(self):
        from decomposition.crawl_gait.crawl_stance import (
            CrawlStanceController as NewStance,
        )

        old_stance = _OldCrawlStanceController(
            phase_length=200,
            stance_ticks=27,
            swing_ticks=22,
            time_step=0.02,
            z_error_constant=0.02,
            body_shift_y=0.06,
        )
        new_stance = NewStance(
            phase_length=200,
            stance_ticks=27,
            swing_ticks=22,
            time_step=0.02,
            z_error_constant=0.02,
            body_shift_y=0.06,
        )

        class MockState:
            def __init__(self):
                self.foot_locations = DEFAULT_STANCE.copy()
                self.robot_height = -0.25

        class MockCommand:
            def __init__(self):
                self.velocity = np.array([0.005, 0.0, 0.0])
                self.yaw_rate = 0.05

        state = MockState()
        command = MockCommand()

        for leg_idx in range(4):
            for first_cycle in [True, False]:
                for move_sideways in [True, False]:
                    for move_left in [True, False]:
                        old_delta_pos, old_delta_ori = old_stance.position_delta(
                            leg_idx,
                            state,
                            command,
                            first_cycle,
                            move_sideways,
                            move_left,
                        )
                        new_delta_pos, new_delta_ori = new_stance.position_delta(
                            leg_idx,
                            state,
                            command,
                            first_cycle,
                            move_sideways,
                            move_left,
                        )
                        np.testing.assert_array_almost_equal(
                            old_delta_pos, new_delta_pos, decimal=15
                        )
                        np.testing.assert_array_almost_equal(
                            old_delta_ori, new_delta_ori, decimal=15
                        )


# ============================================================
# GaitController (базовый)
# ============================================================


class _OldGaitController:
    def __init__(
        self, stance_time, swing_time, time_step, contact_phases, default_stance
    ):
        self.stance_time = stance_time
        self.swing_time = swing_time
        self.time_step = time_step
        self.contact_phases = contact_phases
        self.def_stance = default_stance

    @property
    def default_stance(self):
        return self.def_stance

    @property
    def stance_ticks(self):
        return int(self.stance_time / self.time_step)

    @property
    def swing_ticks(self):
        return int(self.swing_time / self.time_step)

    @property
    def phase_ticks(self):
        temp = []
        for i in range(len(self.contact_phases[0])):
            if 0 in self.contact_phases[:, i]:
                temp.append(self.swing_ticks)
            else:
                temp.append(self.stance_ticks)
        return temp

    @property
    def phase_length(self):
        return sum(self.phase_ticks)

    def phase_index(self, ticks):
        phase_time = ticks % self.phase_length
        phase_sum = 0
        phase_ticks = self.phase_ticks
        for i in range(len(self.contact_phases[0])):
            phase_sum += phase_ticks[i]
            if phase_time < phase_sum:
                return i
        assert False

    def subphase_ticks(self, ticks):
        phase_time = ticks % self.phase_length
        phase_sum = 0
        phase_ticks = self.phase_ticks
        for i in range(len(self.contact_phases[0])):
            phase_sum += phase_ticks[i]
            if phase_time < phase_sum:
                subphase_ticks = phase_time - phase_sum + phase_ticks[i]
                return subphase_ticks
        assert False

    def contacts(self, ticks):
        return self.contact_phases[:, self.phase_index(ticks)]


class TestGaitController:
    def test_phase_properties(self):
        from decomposition.trot_gait.gait_controller import GaitController as NewGait

        contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )
        default_stance = np.array(
            [[0.2, 0.2, -0.18, -0.18], [-0.14, 0.14, -0.14, 0.14], [0, 0, 0, 0]]
        )

        old_gait = _OldGaitController(
            stance_time=0.04,
            swing_time=0.18,
            time_step=0.02,
            contact_phases=contact_phases,
            default_stance=default_stance,
        )
        new_gait = NewGait(
            stance_time=0.04,
            swing_time=0.18,
            time_step=0.02,
            contact_phases=contact_phases,
            default_stance=default_stance,
        )

        assert old_gait.stance_ticks == new_gait.stance_ticks
        assert old_gait.swing_ticks == new_gait.swing_ticks
        assert old_gait.phase_length == new_gait.phase_length
        np.testing.assert_array_equal(old_gait.phase_ticks, new_gait.phase_ticks)

    def test_phase_index(self):
        from decomposition.trot_gait.gait_controller import GaitController as NewGait

        contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )
        default_stance = np.array(
            [[0.2, 0.2, -0.18, -0.18], [-0.14, 0.14, -0.14, 0.14], [0, 0, 0, 0]]
        )

        old_gait = _OldGaitController(
            stance_time=0.04,
            swing_time=0.18,
            time_step=0.02,
            contact_phases=contact_phases,
            default_stance=default_stance,
        )
        new_gait = NewGait(
            stance_time=0.04,
            swing_time=0.18,
            time_step=0.02,
            contact_phases=contact_phases,
            default_stance=default_stance,
        )

        for ticks in [0, 1, 5, 10, 20, 100, 1000]:
            assert old_gait.phase_index(ticks) == new_gait.phase_index(ticks)

    def test_subphase_ticks(self):
        from decomposition.trot_gait.gait_controller import GaitController as NewGait

        contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )
        default_stance = np.array(
            [[0.2, 0.2, -0.18, -0.18], [-0.14, 0.14, -0.14, 0.14], [0, 0, 0, 0]]
        )

        old_gait = _OldGaitController(
            stance_time=0.04,
            swing_time=0.18,
            time_step=0.02,
            contact_phases=contact_phases,
            default_stance=default_stance,
        )
        new_gait = NewGait(
            stance_time=0.04,
            swing_time=0.18,
            time_step=0.02,
            contact_phases=contact_phases,
            default_stance=default_stance,
        )

        for ticks in [0, 1, 5, 10, 20, 100]:
            assert old_gait.subphase_ticks(ticks) == new_gait.subphase_ticks(ticks)

    def test_contacts(self):
        from decomposition.trot_gait.gait_controller import GaitController as NewGait

        contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )
        default_stance = np.array(
            [[0.2, 0.2, -0.18, -0.18], [-0.14, 0.14, -0.14, 0.14], [0, 0, 0, 0]]
        )

        old_gait = _OldGaitController(
            stance_time=0.04,
            swing_time=0.18,
            time_step=0.02,
            contact_phases=contact_phases,
            default_stance=default_stance,
        )
        new_gait = NewGait(
            stance_time=0.04,
            swing_time=0.18,
            time_step=0.02,
            contact_phases=contact_phases,
            default_stance=default_stance,
        )

        for ticks in [0, 1, 5, 10, 20, 100]:
            np.testing.assert_array_equal(
                old_gait.contacts(ticks), new_gait.contacts(ticks)
            )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
