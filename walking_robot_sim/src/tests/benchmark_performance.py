#!/usr/bin/env python3
"""
Benchmark производительности — замер времени выполнения Python функций.
Сравнение old vs new (script) модулей.

Запуск:
    cd /home/redalexdad/GitHub/WalkingRobotSim
    python3 src/tests/benchmark_performance.py
    make test-benchmark
"""

import os
import sys
import timeit

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
PYTHON_SCRIPTS = os.path.abspath(
    os.path.join(TESTS_DIR, "..", "quadropted_controller", "scripts")
)
OLD_DIR = os.path.abspath(os.path.join(TESTS_DIR, "..", "tests", "old"))

ITERATIONS = 5000


def benchmark_old():
    """Замерить время old Python функций."""
    import importlib.util

    sys.path.insert(0, OLD_DIR)

    def load_old(rel):
        p = os.path.join(OLD_DIR, rel)
        spec = importlib.util.spec_from_file_location("m", p)
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        return m

    FK_old_mod = load_old("ForwardKinematics/robot_FK.py")
    IK_old_mod = load_old("InverseKinematics/robot_IK.py")
    Trans_old_mod = load_old("RoboticsUtilities/Transformations.py")

    import numpy as np

    fk_old = FK_old_mod.ForwardKinematics([0.3762, 0.0935], [0.0, 0.0955, 0.213, 0.213])
    ik_old = IK_old_mod.InverseKinematics([0.3762, 0.0935], [0.0, 0.0955, 0.213, 0.213])
    angles = [0, 0.3, -0.6] * 4

    results = {}

    t = timeit.timeit(lambda: Trans_old_mod.rotxyz(0.1, -0.05, 0.02), number=ITERATIONS)
    results["rotxyz"] = t / ITERATIONS * 1000

    def ht_old_test():
        m = Trans_old_mod.homog_transform(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
        return Trans_old_mod.homog_transform_inverse(m.copy())

    t = timeit.timeit(ht_old_test, number=ITERATIONS)
    results["homog_transform_inverse"] = t / ITERATIONS * 1000

    t = timeit.timeit(
        lambda: fk_old.forward_kinematics_all_legs(angles), number=ITERATIONS
    )
    results["FK"] = t / ITERATIONS * 1000

    dx = 0.3762 * 0.5 + 0.02
    dy = 0.0935 * 0.5 + 0.0955
    stance = np.array([[dx, dx, -dx, -dx], [-dy, dy, -dy, dy], [0, 0, 0, 0]])

    def ik_test():
        return ik_old.inverse_kinematics(stance, 0, 0, 0.25, 0, 0, 0)

    t = timeit.timeit(ik_test, number=ITERATIONS)
    results["IK"] = t / ITERATIONS * 1000

    def local_pos_test():
        return ik_old.get_local_positions(stance, 0, 0, 0.25, 0, 0, 0)

    t = timeit.timeit(local_pos_test, number=ITERATIONS)
    results["local_positions"] = t / ITERATIONS * 1000

    return results


def benchmark_new():
    """Замерить время new (script) Python функций."""
    import subprocess

    # Запустить бенчмарк в отдельном процессе с PYTHONPATH
    env = os.environ.copy()
    env["PYTHONPATH"] = PYTHON_SCRIPTS

    script = (
        '''
import sys
sys.path.insert(0, "'''
        + PYTHON_SCRIPTS
        + """")
import timeit
import importlib.util

ITERATIONS = 5000

from ForwardKinematics.forward_kinematics import ForwardKinematics
from RoboticsUtilities.rotation_matrices import rotxyz
from RoboticsUtilities.homogeneous_transforms import homog_transform, homog_transform_inverse
from InverseKinematics.local_positions import compute_local_positions
from InverseKinematics.joint_angles import compute_all_joint_angles

import numpy as np

fk = ForwardKinematics([0.3762, 0.0935], [0.0, 0.0955, 0.213, 0.213])
angles = [0, 0.3, -0.6] * 4
positions = [[0.2, -0.12, -0.2], [0.2, 0.12, -0.2], [-0.2, -0.12, -0.2], [-0.2, 0.12, -0.2]]
lp = np.array([[0.2, 0.2, -0.2, -0.2], [-0.1, 0.1, -0.1, 0.1], [0, 0, 0, 0]])

results = {}

t = timeit.timeit(lambda: rotxyz(0.1, -0.05, 0.02), number=ITERATIONS)
results["rotxyz"] = t / ITERATIONS * 1000

def ht_test():
    m = homog_transform(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    return homog_transform_inverse(m.copy())
t = timeit.timeit(ht_test, number=ITERATIONS)
results["homog_transform_inverse"] = t / ITERATIONS * 1000

t = timeit.timeit(lambda: fk.forward_kinematics_all_legs(angles), number=ITERATIONS)
results["FK"] = t / ITERATIONS * 1000

t = timeit.timeit(lambda: compute_all_joint_angles(positions, 0.0, 0.0955, 0.213, 0.213), number=ITERATIONS)
results["IK"] = t / ITERATIONS * 1000

t = timeit.timeit(lambda: compute_local_positions(lp, 0.3762, 0.0935, 0.01, 0, 0, 0, 0, 0), number=ITERATIONS)
results["local_positions"] = t / ITERATIONS * 1000

for k, v in results.items():
    print(f"{k}:{v}")
"""
    )

    result = subprocess.run(
        ["python3", "-c", script],
        capture_output=True,
        text=True,
        cwd=PYTHON_SCRIPTS,
        env=env,
    )

    results = {}
    if result.returncode == 0:
        for line in result.stdout.strip().split("\n"):
            if ":" in line:
                k, v = line.split(":")
                results[k] = float(v)
    else:
        print(f"ERROR: {result.stderr}")

    return results


def main():
    print("=" * 70)
    print(f"Python Benchmark ({ITERATIONS} итераций)")
    print("=" * 70)

    print("\n[1] Замер old модулей...")
    old_results = benchmark_old()

    print("[2] Замер new (script) модулей...")
    new_results = benchmark_new()

    print("\n" + "=" * 70)
    print("Сводная таблица (время в мс)")
    print("=" * 70)
    print(f"{'Функция':<25} {'Old':>10} {'New':>10} {'Раз':>10} {'x быстрее':>12}")
    print("-" * 70)

    all_keys = sorted(set(old_results.keys()) | set(new_results.keys()))
    total_old = 0
    total_new = 0

    for key in all_keys:
        old_t = old_results.get(key, 0)
        new_t = new_results.get(key, 0)
        if old_t > 0 and new_t > 0:
            ratio = old_t / new_t
            diff = old_t - new_t
            speedup = f"{ratio:.2f}x" if ratio >= 1 else f"{1 / ratio:.2f}x slower"
        else:
            diff = 0
            speedup = "N/A"
        print(f"{key:<25} {old_t:>10.4f} {new_t:>10.4f} {diff:>+10.4f} {speedup:>12}")
        total_old += old_t
        total_new += new_t

    print("-" * 70)
    total_ratio = total_old / total_new if total_new > 0 else 0
    print(
        f"{'ИТОГО':<25} {total_old:>10.4f} {total_new:>10.4f} {total_old - total_new:>+10.4f} {total_ratio:.2f}x быстрее"
    )
    print("=" * 70)

    return 0


if __name__ == "__main__":
    sys.exit(main())
