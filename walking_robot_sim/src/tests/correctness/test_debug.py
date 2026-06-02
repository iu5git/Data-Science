#!/usr/bin/env python3
"""
Comprehensive тест old vs new — запускаем напрямую в том же процессе.
"""

import sys
import os
import math
import numpy as np

BASE_DIR = "/home/redalexdad/GitHub/WalkingRobotSim"
NEW_DIR = BASE_DIR + "/src/quadropted_controller/scripts"
OLD_DIR = BASE_DIR + "/src/tests/old"

# Настраиваем пути - modules должен быть в sys.path чтобы пакеты работали
sys.path.insert(0, "/tmp/test_modules_correct")

# Теперь используем обычные импорты - должны работать
from ForwardKinematics.forward_kinematics import ForwardKinematics as FK_new
from InverseKinematics.inverse_kinematics import InverseKinematics as IK_new
from RoboticsUtilities.rotation_matrices import rotxyz
from RoboticsUtilities.homogeneous_transforms import (
    homog_transform,
    homog_transform_inverse,
)

# Загружаем old модули через exec - используем правильный namespace
fk_ns = {}
exec(open(OLD_DIR + "/ForwardKinematics/robot_FK.py").read(), fk_ns)
FK_old = fk_ns["ForwardKinematics"]

# Debug: проверим что метод доступен
print("DEBUG FK_old methods:", [m for m in dir(FK_old) if not m.startswith("_")])
print("DEBUG forward_kinematics_all_legs:", FK_old.forward_kinematics_all_legs)

ik_ns = {}
exec(open(OLD_DIR + "/InverseKinematics/robot_IK.py").read(), ik_ns)
IK_old = ik_ns["InverseKinematics"]

trans_ns = {}
exec(open(OLD_DIR + "/RoboticsUtilities/Transformations.py").read(), trans_ns)
rotxyz_old = trans_ns["rotxyz"]
homog_transform_old = trans_ns["homog_transform"]
homog_transform_inverse_old = trans_ns["homog_transform_inverse"]

# Создаем экземпляры
FK_new = FK_new([0.3762, 0.0935], [0.0, 0.0955, 0.213, 0.213])
IK_new = IK_new([0.3762, 0.0935], [0.0, 0.0955, 0.213, 0.213])

# Тесты
results = []
passed = 0
failed = 0


def compare(name, old_val, new_val, atol=1e-6):
    global passed, failed
    try:

        def flatten(v):
            if isinstance(v, np.ndarray):
                return v.flatten().tolist()
            if isinstance(v, (list, tuple)):
                return sum([flatten(x) for x in v], [])
            if hasattr(v, "tolist"):
                return v.tolist()
            return [v]

        old_f = flatten(old_val)
        new_f = flatten(new_val)

        if len(old_f) != len(new_f):
            results.append((name, "FAIL", "len %d vs %d" % (len(old_f), len(new_f))))
            failed += 1
            return

        max_diff = max(abs(float(o) - float(n)) for o, n in zip(old_f, new_f))

        if max_diff > atol:
            results.append((name, "FAIL", "max diff %.2e" % max_diff))
            failed += 1
        else:
            results.append((name, "PASS", "max diff %.2e" % max_diff))
            passed += 1
    except Exception as e:
        results.append((name, "ERROR", str(e)[:50]))
        failed += 1


print("\n[1] ForwardKinematics...")
angles = [0, 0.3, -0.6] * 4
compare(
    "FK.forward_kinematics_all_legs",
    FK_old.forward_kinematics_all_legs(angles),
    FK_new.forward_kinematics_all_legs(angles),
)
compare(
    "FK.forward_kinematics_per_leg(0)",
    FK_old.forward_kinematics_per_leg(0, 0.3, -0.6, 0),
    FK_new.forward_kinematics_per_leg(0, 0.3, -0.6, 0),
)
compare(
    "FK.homog_transform",
    FK_old.homog_transform(0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
    FK_new.homog_transform(0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
)

print("[2] InverseKinematics...")
dx = 0.3762 * 0.5 + 0.02
dy = 0.0935 * 0.5 + 0.0955
stance = np.array([[dx, dx, -dx, -dx], [-dy, dy, -dy, dy], [0, 0, 0, 0]])
compare(
    "IK.inverse_kinematics(roll=0)",
    IK_old.inverse_kinematics(stance, 0, 0, 0.25, 0, 0, 0),
    IK_new.inverse_kinematics(stance, 0, 0, 0.25, 0, 0, 0),
)
compare(
    "IK.inverse_kinematics(roll=0.1)",
    IK_old.inverse_kinematics(stance, 0, 0, 0.25, 0.1, 0, 0),
    IK_new.inverse_kinematics(stance, 0, 0, 0.25, 0.1, 0, 0),
)

print("[3] RoboticsUtilities...")
compare(
    "rotxyz(0.1,-0.05,0.02)", rotxyz_old(0.1, -0.05, 0.02), rotxyz(0.1, -0.05, 0.02)
)
compare(
    "rotxyz(π/4,π/6,π/3)",
    rotxyz_old(math.pi / 4, math.pi / 6, math.pi / 3),
    rotxyz(math.pi / 4, math.pi / 6, math.pi / 3),
)
compare(
    "homog_transform",
    homog_transform_old(0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
    homog_transform(0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
)
m = homog_transform(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
compare(
    "homog_transform_inverse",
    homog_transform_inverse_old(m.copy()),
    homog_transform_inverse(m.copy()),
)

print("\n" + "=" * 80)
print("%-55s %-8s %s" % ("TEST", "STATUS", "DETAIL"))
print("-" * 80)
for name, status, detail in results:
    icon = "PASS" if status == "PASS" else "FAIL" if status == "FAIL" else "ERR"
    print("%-55s %-8s %s" % (name, icon, detail))
print("=" * 80)
print("\nTOTAL: %d passed, %d failed" % (passed, failed))
print("=" * 80)
