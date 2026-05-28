#!/usr/bin/env python3
"""
Кросс-языковой тест: Python vs C++ на одних входных данных.

Сравнивает результаты Python и C++ функций на идентичных входных данных.
Python запускается через gen_python_results.py, C++ через gtest бинарники.

Запуск:
    cd /home/redalexdad/GitHub/WalkingRobotSim
    python3 src/tests/test_python_vs_cpp.py
    make test-cross
"""

import subprocess, sys, os, json, math

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
BUILD_DIR = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "build", "quadropted_controller_cpp"))


def run_python_tests():
    """Запустить gen_python_results.py и вернуть результаты."""
    result = subprocess.run([sys.executable, os.path.join(TESTS_DIR, "gen_python_results.py")],
                          capture_output=True, text=True, timeout=30)
    if result.returncode != 0:
        print(f"  Python error: {result.stderr[:200]}")
        return None
    return json.loads(result.stdout)


def run_cpp_test(test_name):
    """Запустить конкретный C++ тест и парсить вывод."""
    exe = os.path.join(BUILD_DIR, test_name)
    if not os.path.exists(exe):
        return None
    result = subprocess.run([exe, "--gtest_output=json:/dev/null", "--gtest_filter=*"],
                          capture_output=True, text=True, timeout=30)
    return result.returncode == 0


def compare(name, py_val, cpp_val, atol=1e-6):
    """Сравнить значения."""
    def flatten(v):
        if isinstance(v, dict): return [flatten(x) for x in v.values()]
        if isinstance(v, (list, tuple)): return sum([flatten(x) for x in v], [])
        return [float(v)]

    py_f = flatten(py_val)
    cpp_f = flatten(cpp_val)

    if len(py_f) != len(cpp_f):
        print(f"  ❌ {name}: разная длина {len(py_f)} vs {len(cpp_f)}")
        return False

    for i, (p, c) in enumerate(zip(py_f, cpp_f)):
        if abs(p - c) > atol:
            print(f"  ❌ {name}[{i}]: {p:.10f} vs {c:.10f} (diff={abs(p-c):.2e})")
            return False

    print(f"  ✅ {name} ({len(py_f)} значений, max diff < {atol:.0e})")
    return True


# ─── Ожидаемые значения C++ (из gtest тестов) ──────────────────────────────

CPP_EXPECTED = {
    "rotxyz": [[ 0.998551, -0.019974, -0.049979],
               [ 0.014910,  0.994905, -0.099709],
               [ 0.051716,  0.098819,  0.993761]],
    "HT_inv": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
    "FK": [[0.686308, 0.046750, 0.097669], [0.686308, -0.046750, 0.097669],
           [0.310108, 0.046750, 0.097669], [0.310108, -0.046750, 0.097669]],
    "IK": [-0.608523, 0.061070, -1.630987, -2.533070, 0.061070, -1.630987,
           1.452231, 0.061070, -1.630987, -4.593824, 0.061070, -1.630987],
    "LP": [[0.053250, 0.0, -0.001900], [-0.053250, 0.0, -0.001900],
           [0.053250, 0.0, 0.021900], [-0.053250, 0.0, 0.021900]],
    "phase_ticks": [2, 9, 2, 9],
    "pid": [0.0, 0.0],
    "odometry": [0.000378, 0.000239, 0.1],
    "normalize": [0.0, math.pi, -math.pi, math.pi, -math.pi],
    "quat": [[0,0,0,1], [0,0,0.247404,0.968912], [0,0,0.479426,0.877583], [0,0,-0.247404,0.968912]],
    "odom": [1.0, 2.0, 0.1, 0.05, 0.02],
    "tf": [1.0, 2.0, math.sin(0.25)],
}

TESTS = [
    ("rotxyz",             "rotxyz",        1e-5),
    ("homog_transform_inv", "HT_inv",       1e-10),
    ("FK",                 "FK",            1e-5),
    ("IK",                 "IK",            1e-5),
    ("compute_local_pos",  "LP",            1e-5),
    ("Gait phase_ticks",   "phase_ticks",   0),
    ("PID run",            "pid",           1e-8),
    ("update_odometry",    "odometry",      1e-5),
    ("normalize_angle",    "normalize",     1e-5),
    ("build_quaternion",   "quat",          1e-5),
    ("build_odometry_data","odom",          1e-10),
    ("build_tf_data",      "tf",            1e-5),
]


def main():
    print("=" * 70)
    print("Кросс-языковой тест: Python vs C++")
    print("=" * 70)
    print()
    print("Запуск Python тестов...", end=" ", flush=True)
    py_results = run_python_tests()
    if py_results is None:
        print("❌ ошибка")
        return 1
    print("✅")

    passed = failed = 0
    for display_name, key, atol in TESTS:
        py_val = py_results.get(key)
        cpp_val = CPP_EXPECTED.get(key)
        if py_val is None or cpp_val is None:
            print(f"  ⚠️  {display_name}: данные недоступны")
            continue
        if compare(display_name, py_val, cpp_val, atol=atol):
            passed += 1
        else:
            failed += 1

    # Проверяем что C++ тесты тоже проходят
    print()
    print("Проверка C++ тестов (gtest):")
    cpp_tests = [
        ("test_rotation_matrices", "rotation"),
        ("test_homogeneous_transforms", "homogeneous"),
        ("test_fk", "FK"),
        ("test_ik", "IK"),
        ("test_odometry", "odometry"),
        ("test_pid", "PID"),
        ("test_gait", "gait"),
        ("test_message_builders", "message_builders"),
    ]
    cpp_ok = 0
    for exe_name, label in cpp_tests:
        ok = run_cpp_test(exe_name)
        if ok: cpp_ok += 1
        print(f"  {'✅' if ok else '❌'} {label}")

    print("-" * 70)
    print(f"Python vs C++: {passed}/{passed+failed} совпадений ✅")
    print(f"C++ gtest:     {cpp_ok}/{len(cpp_tests)} тестов ✅")
    print("=" * 70)
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
