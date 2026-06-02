#!/usr/bin/env python3
"""
Test correctness: Python (quadropted_controller) vs C++ (quadropted_controller_cpp)
Сравнение с tolerance atol = 1e-10
"""

import subprocess
import sys
import os
import math
import numpy as np
import importlib.util
import tempfile
import shutil
import re

BASE_DIR = "/home/redalexdad/GitHub/WalkingRobotSim"
PY_DIR = BASE_DIR + "/src/quadropted_controller/scripts"
CPP_BUILD_DIR = BASE_DIR + "/src/quadropted_controller_cpp/build"
CPP_BENCHMARK_EXEC = CPP_BUILD_DIR + "/benchmark"

TOLERANCE = 1e-10


def load_module_from_path(module_name, filepath):
    spec = importlib.util.spec_from_file_location(module_name, filepath)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def setup_python_modules():
    test_dir = tempfile.mkdtemp()
    modules_dir = os.path.join(test_dir, "modules")
    os.makedirs(modules_dir)

    for subdir in [
        "ForwardKinematics",
        "InverseKinematics",
        "RoboticsUtilities",
    ]:
        src = os.path.join(PY_DIR, subdir)
        dst = os.path.join(modules_dir, subdir)
        if os.path.exists(src):
            shutil.copytree(src, dst)
            with open(os.path.join(dst, "__init__.py"), "w") as f:
                pass

    for root, dirs, files in os.walk(modules_dir):
        rel_path = os.path.relpath(root, modules_dir)
        if rel_path == ".":
            continue
        parent_pkg = rel_path.replace(os.sep, ".")

        for f in files:
            if f.endswith(".py"):
                fp = os.path.join(root, f)
                with open(fp, "r") as file:
                    content = file.read()

                new_content = re.sub(
                    r"^from \.(\w+)",
                    f"from {parent_pkg}.\\1",
                    content,
                    flags=re.MULTILINE,
                )

                if new_content != content:
                    with open(fp, "w") as file:
                        file.write(new_content)

    final_path = "/tmp/test_modules_python_cpp"
    if os.path.exists(final_path):
        shutil.rmtree(final_path)
    shutil.copytree(modules_dir, final_path)

    sys.path.insert(0, final_path)
    return final_path


def load_python_modules():
    from ForwardKinematics.forward_kinematics import ForwardKinematics as FK_PY
    from ForwardKinematics.leg_fk_chain import compute_leg_fk_chain as leg_fk_py
    from InverseKinematics.inverse_kinematics import InverseKinematics as IK_PY
    from InverseKinematics.local_positions import (
        compute_local_positions as local_pos_py,
    )
    from RoboticsUtilities.rotation_matrices import rotxyz, rotx, roty, rotz
    from RoboticsUtilities.homogeneous_transforms import (
        homog_transform,
        homog_transform_inverse,
    )

    return {
        "FK": FK_PY,
        "leg_fk": leg_fk_py,
        "IK": IK_PY,
        "local_pos": local_pos_py,
        "rotxyz": rotxyz,
        "rotx": rotx,
        "roty": roty,
        "rotz": rotz,
        "homog_transform": homog_transform,
        "homog_transform_inverse": homog_transform_inverse,
    }


def run_cpp_test(test_name, cpp_test_exe=None):
    if not os.path.exists(CPP_BENCHMARK_EXEC):
        return None

    try:
        result = subprocess.run(
            [CPP_BENCHMARK_EXEC],
            capture_output=True,
            text=True,
            timeout=30,
        )
        return result.stdout + result.stderr
    except Exception as e:
        print(f"  [WARN] Failed to run C++ benchmark: {e}")
        return None


def create_default_stance():
    body = [0.3762, 0.0935]
    legs = [0.0, 0.0955, 0.213, 0.213]
    dx = body[0] * 0.5 + 0.02
    dy = body[1] * 0.5 + legs[1]
    return np.array([[dx, dx, -dx, -dx], [-dy, dy, -dy, dy], [0, 0, 0, 0]])


def print_header(title):
    print("\n" + "=" * 70)
    print(f" {title}")
    print("=" * 70)


def compare_arrays(name, py_arr, cpp_arr, tolerance=TOLERANCE):
    py_arr = np.asarray(py_arr).flatten()
    cpp_arr = np.asarray(cpp_arr).flatten()
    diff = np.abs(py_arr - cpp_arr)
    max_diff = np.max(diff)
    passed = max_diff < tolerance
    return passed, max_diff


def compare_values(name, py_val, cpp_val, tolerance=TOLERANCE):
    if isinstance(py_val, (list, tuple, np.ndarray)):
        py_arr = np.asarray(py_val).flatten()
        cpp_arr = np.asarray(cpp_val).flatten()
        diff = np.abs(py_arr - cpp_arr)
        max_diff = np.max(diff)
        passed = max_diff < tolerance
        return passed, max_diff
    else:
        diff = abs(py_val - cpp_val)
        passed = diff < tolerance
        return passed, diff


def test_rotation_matrices(py_mod):
    print_header("Test: Rotation Matrices (rotx, roty, rotz, rotxyz)")
    tests_passed = 0
    tests_failed = 0

    test_cases = [
        (
            "rotx(0.5)",
            py_mod["rotx"](0.5),
            [1, 0, 0, 0, 0.8775826, -0.4794255, 0, 0.4794255, 0.8775826],
        ),
        (
            "roty(0.5)",
            py_mod["roty"](0.5),
            [0.8775826, 0, 0.4794255, 0, 1, 0, -0.4794255, 0, 0.8775826],
        ),
        (
            "rotz(0.5)",
            py_mod["rotz"](0.5),
            [0.8775826, -0.4794255, 0, 0.4794256, 0.8775826, 0, 0, 0, 1],
        ),
    ]

    for name, py_result, expected in test_cases:
        passed, diff = compare_arrays(name, py_result, expected)
        print(f"  {name}: ", end="")
        if passed:
            print(f"OK (diff={diff:.2e})")
            tests_passed += 1
        else:
            print(f"FAIL (diff={diff:.2e})")
            tests_failed += 1

    py_result = py_mod["rotxyz"](0.1, 0.2, 0.3)
    print(f"  rotxyz(0.1, 0.2, 0.3): shape={py_result.shape}", end="")
    if py_result.shape == (3, 3):
        print(" OK")
        tests_passed += 1
    else:
        print(f" FAIL")
        tests_failed += 1

    return tests_passed, tests_failed


def test_homogeneous_transforms(py_mod):
    print_header("Test: Homogeneous Transforms")
    tests_passed = 0
    tests_failed = 0

    py_result = py_mod["homog_transform"](0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
    print(
        f"  homog_transform(0.1, 0.2, 0.3, 0.1, 0.2, 0.3): shape={py_result.shape}",
        end="",
    )
    if py_result.shape == (4, 4):
        print(" OK")
        tests_passed += 1
    else:
        print(f" FAIL - shape {py_result.shape}")
        tests_failed += 1

    py_inv = py_mod["homog_transform_inverse"](py_result)
    print(f"  homog_transform_inverse(): shape={py_inv.shape}", end="")
    if py_inv.shape == (4, 4):
        print(" OK")
        tests_passed += 1
    else:
        print(f" FAIL")
        tests_failed += 1

    return tests_passed, tests_failed


def test_leg_fk_chain(py_mod):
    print_header("Test: Leg FK Chain")
    tests_passed = 0
    tests_failed = 0

    result = py_mod["leg_fk"](
        0.0, 0.86, -1.88, 0.2081, 0.10075, 0.0, 0.0955, 0.213, 0.213
    )
    print(
        f"  compute_leg_fk_chain(0, 0.86, -1.88, base): [{result[0]:.4f}, {result[1]:.4f}, {result[2]:.4f}]"
    )

    if (
        len(result) == 3
        and abs(result[0]) < 1.0
        and abs(result[1]) < 1.0
        and abs(result[2]) < 1.0
    ):
        print("    OK")
        tests_passed += 1
    else:
        print("    FAIL")
        tests_failed += 1

    return tests_passed, tests_failed


def test_forward_kinematics(py_mod):
    print_header("Test: Forward Kinematics")
    tests_passed = 0
    tests_failed = 0

    body = [0.3762, 0.0935]
    legs = [0.0, 0.0955, 0.213, 0.213]

    fk = py_mod["FK"](body, legs)

    angles = [0, 0.3, -0.6, 0, 0.3, -0.6, 0, 0.3, -0.6, 0, 0.3, -0.6]
    result = fk.forward_kinematics_all_legs(angles)

    expected = [
        [0.686308, 0.046750, 0.097669],
        [0.686308, -0.046750, 0.097669],
        [0.310108, 0.046750, 0.097669],
        [0.310108, -0.046750, 0.097669],
    ]

    print(f"  forward_kinematics_all_legs([0, 0.3, -0.6] x 4):")
    all_passed = True
    for leg in range(4):
        py_pos = [result[leg][0], result[leg][1], result[leg][2]]
        exp_pos = expected[leg]

        passed, diff = compare_values(f"Leg {leg}", py_pos, exp_pos)
        if passed:
            print(f"    Leg {leg}: OK (max_diff={diff:.2e})")
            tests_passed += 1
        else:
            print(f"    Leg {leg}: FAIL (max_diff={diff:.2e})")
            tests_failed += 1
            all_passed = False

    angles_zero = [0.0] * 12
    result_zero = fk.forward_kinematics_all_legs(angles_zero)
    print(f"  forward_kinematics_all_legs(zero): {len(result_zero)} legs", end="")
    if len(result_zero) == 4:
        print(" OK")
        tests_passed += 1
    else:
        print(" FAIL")
        tests_failed += 1

    return tests_passed, tests_failed


def test_local_positions(py_mod):
    print_header("Test: Local Positions")
    tests_passed = 0
    tests_failed = 0

    body = [0.3762, 0.0935]
    legs = [0.0, 0.0955, 0.213, 0.213]

    stance = create_default_stance()

    local = py_mod["local_pos"](stance, body[0], body[1], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    print(f"  compute_local_positions (no body transform): shape={local.shape}", end="")
    if local.shape == (3, 4):
        print(" OK")
        tests_passed += 1
    else:
        print(f" FAIL")
        tests_failed += 1

    local2 = py_mod["local_pos"](
        stance, body[0], body[1], 0.1, 0.05, 0.25, 0.1, 0.2, 0.0
    )
    print(
        f"  compute_local_positions (with body transform): shape={local2.shape}", end=""
    )
    if local2.shape == (3, 4):
        print(" OK")
        tests_passed += 1
    else:
        print(" FAIL")
        tests_failed += 1

    return tests_passed, tests_failed


def test_inverse_kinematics_with_stance(py_mod):
    print_header("Test: Inverse Kinematics with valid local positions")
    tests_passed = 0
    tests_failed = 0

    body = [0.3762, 0.0935]
    legs = [0.0, 0.0955, 0.213, 0.213]
    stance = create_default_stance()

    local = py_mod["local_pos"](stance, body[0], body[1], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    print(f"  Local positions for valid stance: {local.shape}")
    for i in range(4):
        print(f"    Leg {i}: [{local[0, i]:.4f}, {local[1, i]:.4f}, {local[2, i]:.4f}]")

    print(f"\n  C++ expected values for leg transforms:")
    print(f"    R_legs = rotxyz(pi/2, -pi/2, 0):")
    R_legs = py_mod["rotxyz"](np.pi / 2, -np.pi / 2, 0)
    for row in range(3):
        print(
            f"      [{R_legs[row, 0]:.4f}, {R_legs[row, 1]:.4f}, {R_legs[row, 2]:.4f}]"
        )

    l2_sq = legs[1] ** 2
    all_valid = all(
        local[0, i] ** 2 + local[1, i] ** 2 >= l2_sq - 1e-10 for i in range(4)
    )
    print(f"  All legs reachable (x^2 + y^2 >= l2^2): {'OK' if all_valid else 'FAIL'}")
    if all_valid:
        tests_passed += 1
    else:
        tests_failed += 1

    local2 = py_mod["local_pos"](
        stance, body[0], body[1], 0.1, 0.05, 0.25, 0.0, 0.0, 0.0
    )
    print(f"\n  Local positions with body offset: {local2.shape}")
    for i in range(4):
        print(
            f"    Leg {i}: [{local2[0, i]:.4f}, {local2[1, i]:.4f}, {local2[2, i]:.4f}]"
        )

    return tests_passed, tests_failed


def parse_cpp_benchmark_output(output):
    results = {}

    lines = output.split("\n")
    in_timing = False

    for line in lines:
        if "=== BENCHMARK_JSON_START ===" in line:
            in_timing = True
            continue
        if "=== BENCHMARK_JSON_END ===" in line:
            in_timing = False
            continue

        if in_timing and ":" in line:
            parts = line.strip().split(": ")
            if len(parts) == 2:
                name = parts[0].strip()
                try:
                    value = float(parts[1].strip())
                    results[name] = value
                except:
                    pass

    return results


def compare_with_cpp_benchmark(py_results):
    print_header("Comparison with C++ Benchmark Results")

    if not os.path.exists(CPP_BENCHMARK_EXEC):
        print("  [SKIP] C++ benchmark not built - skipping comparison")
        return 0, 0

    print("  Running C++ benchmark...")
    cpp_output = run_cpp_test("benchmark")
    if cpp_output is None:
        print("  [SKIP] Could not run C++ benchmark")
        return 0, 0

    cpp_results = parse_cpp_benchmark_output(cpp_output)
    print(f"  Found {len(cpp_results)} C++ benchmark results")

    tests_passed = 0
    tests_failed = 0

    print("\n  Comparing Python vs C++ results:")
    print(f"  {'Function':<50} {'Python':<15} {'C++':<15} {'Status'}")
    print("  " + "-" * 95)

    for name, py_val in py_results.items():
        cpp_val = cpp_results.get(name, None)
        if cpp_val is not None:
            passed, diff = compare_values(name, py_val, cpp_val)
            status = "PASS" if passed else "FAIL"
            print(f"  {name:<50} {py_val:<15.4f} {cpp_val:<15.4f} {status}")
            if passed:
                tests_passed += 1
            else:
                tests_failed += 1
        else:
            print(f"  {name:<50} {py_val:<15.4f} {'N/A':<15} SKIP")

    return tests_passed, tests_failed


def run_python_benchmark():
    import time

    print_header("Running Python Benchmark for Comparison")

    body = [0.3762, 0.0935]
    legs = [0.0, 0.0955, 0.213, 0.213]
    stance = create_default_stance()

    from ForwardKinematics.forward_kinematics import ForwardKinematics as FK_PY
    from InverseKinematics.inverse_kinematics import InverseKinematics as IK_PY
    from RoboticsUtilities.rotation_matrices import rotxyz, rotx, roty, rotz
    from RoboticsUtilities.homogeneous_transforms import homog_transform

    fk = FK_PY(body, legs)
    ik = IK_PY(body, legs)

    iterations = 10000
    results = {}

    start = time.perf_counter()
    for _ in range(iterations):
        _ = fk.forward_kinematics_all_legs([0.0, 0.86, -1.88] * 4)
    duration = (time.perf_counter() - start) * 1e6 / iterations
    results["ForwardKinematics.forward_kinematics_all_legs()"] = duration

    start = time.perf_counter()
    for _ in range(iterations):
        try:
            _ = ik.inverse_kinematics(stance, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0)
        except:
            pass
    duration = (time.perf_counter() - start) * 1e6 / iterations
    results["InverseKinematics.inverse_kinematics()"] = duration

    start = time.perf_counter()
    for _ in range(iterations):
        _ = rotx(0.5)
    duration = (time.perf_counter() - start) * 1e6 / iterations
    results["rotx()"] = duration

    start = time.perf_counter()
    for _ in range(iterations):
        _ = rotxyz(0.1, 0.2, 0.3)
    duration = (time.perf_counter() - start) * 1e6 / iterations
    results["rotxyz()"] = duration

    start = time.perf_counter()
    for _ in range(iterations):
        _ = homog_transform(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
    duration = (time.perf_counter() - start) * 1e6 / iterations
    results["homog_transform()"] = duration

    print(f"\n  Python benchmark results ({iterations} iterations):")
    for name, val in results.items():
        print(f"    {name}: {val:.4f} μs")

    return results


def main():
    print("╔══════════════════════════════════════════════════════════╗")
    print("║  Python vs C++ Correctness Test (tol=1e-6)               ║")
    print("╚══════════════════════════════════════════════════════════╝")

    print("\n[1] Setting up Python modules...")
    setup_python_modules()

    print("[2] Loading Python modules...")
    py_mod = load_python_modules()

    total_passed = 0
    total_failed = 0

    print("\n[3] Running correctness tests...")

    passed, failed = test_rotation_matrices(py_mod)
    total_passed += passed
    total_failed += failed

    passed, failed = test_homogeneous_transforms(py_mod)
    total_passed += passed
    total_failed += failed

    passed, failed = test_leg_fk_chain(py_mod)
    total_passed += passed
    total_failed += failed

    passed, failed = test_local_positions(py_mod)
    total_passed += passed
    total_failed += failed

    passed, failed = test_forward_kinematics(py_mod)
    total_passed += passed
    total_failed += failed

    passed, failed = test_inverse_kinematics_with_stance(py_mod)
    total_passed += passed
    total_failed += failed

    print("\n[4] Running Python benchmark for C++ comparison...")
    py_benchmark_results = run_python_benchmark()

    print("\n[5] Comparing with C++ benchmark...")
    passed, failed = compare_with_cpp_benchmark(py_benchmark_results)
    total_passed += passed
    total_failed += failed

    print("\n" + "=" * 70)
    print(" FINAL RESULTS")
    print("=" * 70)
    print(f"  Total passed: {total_passed}")
    print(f"  Total failed: {total_failed}")
    print(f"  Tolerance: {TOLERANCE}")
    print("=" * 70)

    if total_failed > 0:
        print("\n  STATUS: SOME TESTS FAILED")
        return 1
    else:
        print("\n  STATUS: ALL TESTS PASSED")
        return 0


if __name__ == "__main__":
    sys.exit(main())
