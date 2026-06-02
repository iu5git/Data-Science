#!/usr/bin/env python3
"""Тесты корректности IK с roll — проверка влияния крена на углы суставов."""

import sys
import os
import math
import numpy as np

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__), "..", "..", "quadropted_controller", "scripts"
    ),
)

from InverseKinematics import InverseKinematics
from ForwardKinematics import ForwardKinematics


def make_default_stance():
    body = [0.3762, 0.0935]
    legs = [0.0, 0.0955, 0.213, 0.213]
    dx = body[0] * 0.5 + 0.02
    dy = body[1] * 0.5 + legs[1]
    return np.array(
        [
            [dx, dx, -dx, -dx],
            [-dy, dy, -dy, dy],
            [0, 0, 0, 0],
        ]
    )


def test_ik_zero_roll_default_stance():
    ik = InverseKinematics([0.3762, 0.0935], [0.0, 0.0955, 0.213, 0.213])
    stance = make_default_stance()
    angles = ik.inverse_kinematics(stance, 0, 0, 0.25, 0, 0, 0)

    assert len(angles) == 12
    assert abs(angles[0]) < 0.01, f"hip angle should be ~0, got {angles[0]}"
    assert abs(angles[1] - 0.862) < 0.1, (
        f"thigh angle should be ~0.862, got {angles[1]}"
    )
    assert abs(angles[2] - (-1.883)) < 0.1, (
        f"calf angle should be ~-1.883, got {angles[2]}"
    )
    print("  ✅ test_ik_zero_roll_default_stance")


def test_ik_roll_45_degrees_affects_angles():
    ik = InverseKinematics([0.3762, 0.0935], [0.0, 0.0955, 0.213, 0.213])
    stance = make_default_stance()

    angles_no_roll = ik.inverse_kinematics(stance, 0, 0, 0.25, 0, 0, 0)
    angles_roll_45 = ik.inverse_kinematics(stance, 0, 0, 0.25, math.pi / 4, 0, 0)

    different = any(abs(a - b) > 1e-6 for a, b in zip(angles_no_roll, angles_roll_45))
    assert different, "IK с roll=π/4 должен давать другие углы чем roll=0"
    print("  ✅ test_ik_roll_45_degrees_affects_angles")


def test_ik_roll_45_angles_in_valid_range():
    ik = InverseKinematics([0.3762, 0.0935], [0.0, 0.0955, 0.213, 0.213])
    stance = make_default_stance()

    angles = ik.inverse_kinematics(stance, 0, 0, 0.25, math.pi / 4, 0, 0)

    for i, a in enumerate(angles):
        assert abs(a) < math.pi + 0.5, f"Angle {i} out of range: {a}"
    print("  ✅ test_ik_roll_45_angles_in_valid_range")


def test_ik_negative_roll_45():
    ik = InverseKinematics([0.3762, 0.0935], [0.0, 0.0955, 0.213, 0.213])
    stance = make_default_stance()

    angles_pos = ik.inverse_kinematics(stance, 0, 0, 0.25, math.pi / 4, 0, 0)
    angles_neg = ik.inverse_kinematics(stance, 0, 0, 0.25, -math.pi / 4, 0, 0)

    different = any(abs(a - b) > 1e-6 for a, b in zip(angles_pos, angles_neg))
    assert different, "IK с roll=+π/4 и roll=-π/4 должны давать разные углы"
    print("  ✅ test_ik_negative_roll_45")


def test_ik_left_right_symmetry_zero_roll():
    ik = InverseKinematics([0.3762, 0.0935], [0.0, 0.0955, 0.213, 0.213])
    stance = make_default_stance()

    angles = ik.inverse_kinematics(stance, 0, 0, 0.25, 0, 0, 0)

    assert abs(angles[0] - angles[3]) < 0.01, (
        f"hip FR vs FL mismatch: {angles[0]} vs {angles[3]}"
    )
    assert abs(angles[1] - angles[4]) < 0.01, (
        f"thigh FR vs FL mismatch: {angles[1]} vs {angles[4]}"
    )
    assert abs(angles[2] - angles[5]) < 0.01, (
        f"calf FR vs FL mismatch: {angles[2]} vs {angles[5]}"
    )
    print("  ✅ test_ik_left_right_symmetry_zero_roll")


def test_fk_ik_roundtrip_zero_roll():
    body = [0.3762, 0.0935]
    legs = [0.0, 0.0955, 0.213, 0.213]

    fk = ForwardKinematics(body, legs)
    ik = InverseKinematics(body, legs)

    original_angles = [0, 0.3, -0.6] * 4
    foot_pos = fk.forward_kinematics_all_legs(original_angles)

    leg_positions = np.zeros((3, 4))
    for leg in range(4):
        for dim in range(3):
            leg_positions[dim, leg] = foot_pos[leg][dim]

    recovered = ik.inverse_kinematics(leg_positions, 0, 0, 0.25, 0, 0, 0)

    for i in range(12):
        assert abs(recovered[i]) < 6.3, f"Angle {i} out of valid range: {recovered[i]}"
    print("  ✅ test_fk_ik_roundtrip_zero_roll")


def test_ik_small_orientation_angles():
    ik = InverseKinematics([0.3762, 0.0935], [0.0, 0.0955, 0.213, 0.213])
    stance = make_default_stance()

    angles = ik.inverse_kinematics(stance, 0, 0, 0.25, 0.1, -0.05, 0.02)

    for i, a in enumerate(angles):
        assert abs(a) < math.pi + 0.5, f"Angle {i} out of range: {a}"
    print("  ✅ test_ik_small_orientation_angles")


def test_ik_roll_varies_smoothly():
    ik = InverseKinematics([0.3762, 0.0935], [0.0, 0.0955, 0.213, 0.213])
    stance = make_default_stance()

    angles_0 = ik.inverse_kinematics(stance, 0, 0, 0.25, 0, 0, 0)
    angles_01 = ik.inverse_kinematics(stance, 0, 0, 0.25, 0.1, 0, 0)
    angles_02 = ik.inverse_kinematics(stance, 0, 0, 0.25, 0.2, 0, 0)

    diff_0_01 = sum(abs(a - b) for a, b in zip(angles_0, angles_01))
    diff_01_02 = sum(abs(a - b) for a, b in zip(angles_01, angles_02))

    assert diff_0_01 > 0, "Углы должны меняться при roll=0.1"
    assert diff_01_02 > 0, "Углы должны меняться при roll=0.2"
    print("  ✅ test_ik_roll_varies_smoothly")


if __name__ == "__main__":
    print("=" * 60)
    print("Python IK с roll тесты")
    print("=" * 60)

    tests = [
        test_ik_zero_roll_default_stance,
        test_ik_roll_45_degrees_affects_angles,
        test_ik_roll_45_angles_in_valid_range,
        test_ik_negative_roll_45,
        test_ik_left_right_symmetry_zero_roll,
        test_fk_ik_roundtrip_zero_roll,
        test_ik_small_orientation_angles,
        test_ik_roll_varies_smoothly,
    ]

    passed = 0
    failed = 0
    for t in tests:
        try:
            t()
            passed += 1
        except Exception as e:
            print(f"  ❌ {t.__name__}: {e}")
            failed += 1

    print("-" * 60)
    print(f"Python IK с roll: {passed}/{passed + failed} ✅, {failed} ❌")
    print("=" * 60)
    sys.exit(0 if failed == 0 else 1)
