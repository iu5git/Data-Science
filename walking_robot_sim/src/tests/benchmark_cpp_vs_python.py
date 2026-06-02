#!/usr/bin/env python3
"""
C++ Benchmark: замеряет производительность C++ и сравнивает с Python.

Запускает C++ бенчмарк (если скомпилирован) или замеряет напрямую.
Сравнивает с результатами Python benchmark_performance.py.

Запуск:
    cd /home/redalexdad/GitHub/WalkingRobotSim
    python3 src/tests/benchmark_cpp_vs_python.py
    make bench-cpp
"""

import subprocess
import sys
import os
import json
import timeit
import math

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
PYTHON_SCRIPTS = os.path.abspath(os.path.join(TESTS_DIR, "..", "quadropted_controller", "scripts"))
BUILD_DIR = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "build", "quadropted_controller_cpp"))
ITERATIONS = 5000

# ════════════════════════════════════════════════════════════
# Python Benchmark
# ════════════════════════════════════════════════════════════

def benchmark_python():
    """Замерить время Python функций."""
    sys.path.insert(0, PYTHON_SCRIPTS)
    from ForwardKinematics.forward_kinematics import ForwardKinematics
    from RoboticsUtilities.rotation_matrices import rotxyz
    from RoboticsUtilities.homogeneous_transforms import homog_transform, homog_transform_inverse
    from InverseKinematics.local_positions import compute_local_positions
    from InverseKinematics.joint_angles import compute_all_joint_angles
    from QuadrupedOdometry.odometry_state import OdometryState
    from QuadrupedOdometry.odometry_update import update_odometry

    # Прямой импорт минуя RobotController/__init__.py
    import importlib.util
    def _load(rel):
        p = os.path.join(PYTHON_SCRIPTS, rel)
        spec = importlib.util.spec_from_file_location("m", p)
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        return m
    GaitController = _load("RobotController/GaitController.py").GaitController
    import numpy as np

    # Тестовые данные
    fk = ForwardKinematics([0.3762, 0.0935], [0.0, 0.0955, 0.213, 0.213])
    angles = [0, 0.3, -0.6] * 4
    positions = [[0.2, -0.12, -0.2], [0.2, 0.12, -0.2], [-0.2, -0.12, -0.2], [-0.2, 0.12, -0.2]]
    lp = np.array([[0.2, 0.2, -0.2, -0.2], [-0.1, 0.1, -0.1, 0.1], [0, 0, 0, 0]])
    cp = np.array([[1,1,1,0],[1,0,1,1],[1,0,1,1],[1,1,1,0]])
    gc = GaitController(0.04, 0.18, 0.02, cp, np.zeros((3, 4)))

    state = OdometryState()
    state.linear_velocity_x = 0.02
    state.linear_velocity_y = 0.01
    state.theta = 0.1
    state.foot_contacts = [False] * 4

    results = {}

    # rotxyz
    t = timeit.timeit(lambda: rotxyz(0.1, -0.05, 0.02), number=ITERATIONS)
    results["rotxyz"] = t / ITERATIONS * 1000  # мс

    # homog_transform + inverse
    def ht_test():
        m = homog_transform(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
        return homog_transform_inverse(m.copy())
    t = timeit.timeit(ht_test, number=ITERATIONS)
    results["homog_transform_inverse"] = t / ITERATIONS * 1000

    # FK
    t = timeit.timeit(lambda: fk.forward_kinematics_all_legs(angles), number=ITERATIONS)
    results["FK"] = t / ITERATIONS * 1000

    # IK
    t = timeit.timeit(lambda: compute_all_joint_angles(positions, 0.0, 0.0955, 0.213, 0.213), number=ITERATIONS)
    results["IK"] = t / ITERATIONS * 1000

    # local_positions
    t = timeit.timeit(lambda: compute_local_positions(lp, 0.3762, 0.0935, 0.01, 0, 0, 0, 0, 0), number=ITERATIONS)
    results["local_positions"] = t / ITERATIONS * 1000

    # Gait phase_ticks
    t = timeit.timeit(lambda: gc.phase_ticks, number=ITERATIONS)
    results["GaitController.phase_ticks"] = t / ITERATIONS * 1000

    # update_odometry
    def odometry_test():
        s = OdometryState()
        s.linear_velocity_x = 0.02
        s.linear_velocity_y = 0.01
        s.theta = 0.1
        s.foot_contacts = [False] * 4
        update_odometry(s, 0.02)
    t = timeit.timeit(odometry_test, number=ITERATIONS)
    results["update_odometry"] = t / ITERATIONS * 1000

    return results


# ════════════════════════════════════════════════════════════
# C++ Benchmark
# ════════════════════════════════════════════════════════════

def benchmark_cpp():
    """Замерить время C++ функций через встроенный бенчмарк."""
    # Проверяем есть ли скомпилированный benchmark
    bench_exe = os.path.join(BUILD_DIR, "benchmark_cpp")
    if os.path.exists(bench_exe):
        result = subprocess.run([bench_exe, str(ITERATIONS)], capture_output=True, text=True, timeout=60)
        if result.returncode == 0:
            return json.loads(result.stdout)

    # Fallback: замеряем через C++ тесты
    print("  ⚠️  benchmark_cpp не найден, собираю...")
    return None


# ════════════════════════════════════════════════════════════
# C++ Benchmark Source Generator
# ════════════════════════════════════════════════════════════

CPP_BENCH_SOURCE = r'''
#include <iostream>
#include <chrono>
#include <vector>
#include <Eigen/Dense>
#include "quadropted_controller_cpp/utils/rotation_matrices.hpp"
#include "quadropted_controller_cpp/utils/homogeneous_transforms.hpp"
#include "quadropted_controller_cpp/kinematics/forward_kinematics.hpp"
#include "quadropted_controller_cpp/kinematics/inverse_kinematics.hpp"
#include "quadropted_controller_cpp/controllers/gait_controller.hpp"
#include "quadropted_controller_cpp/odometry/odometry.hpp"

static std::vector<double> vd(std::initializer_list<double> l){ return std::vector<double>(l); }

int main(int argc, char** argv) {
    int iterations = 5000;
    if (argc > 1) iterations = std::atoi(argv[1]);

    auto bench = [&](const char* name, auto&& fn) {
        auto start = std::chrono::high_resolution_clock::now();
        volatile double sink = 0;
        for (int i = 0; i < iterations; ++i) {
            auto r = fn();
            sink += r.empty() ? 0 : r[0];
        }
        auto end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end - start).count() / iterations;
        printf("\"%s\": %.6f,\n", name, ms);
        (void)sink;
    };

    // rotxyz
    bench("rotxyz", []{
        auto m = quadropted::rotxyz(0.1, -0.05, 0.02);
        return vd({m(0,0), m(1,1), m(2,2)});
    });

    // homog_transform + inverse
    bench("homog_transform_inverse", []{
        auto m = quadropted::homog_transform(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
        auto inv = quadropted::homog_transform_inverse(m);
        return vd({inv(0,0), inv(1,1), inv(2,2), inv(3,3)});
    });

    // FK
    {
        quadropted::ForwardKinematics fk(0.3762, 0.0935, 0.0, 0.0955, 0.213, 0.213);
        std::vector<double> angles = {0, 0.3, -0.6, 0, 0.3, -0.6, 0, 0.3, -0.6, 0, 0.3, -0.6};
        bench("FK", [&]{
            auto r = fk.forward_kinematics_all_legs(angles);
            return vd({r[0].x(), r[1].x(), r[2].x(), r[3].x()});
        });
    }

    // IK
    {
        Eigen::MatrixXd pos(3, 4);
        pos << 0.2, 0.2, -0.2, -0.2, -0.12, 0.12, -0.12, 0.12, -0.2, -0.2, -0.2, -0.2;
        bench("IK", [&]{
            auto r = quadropted::compute_all_joint_angles(pos, 0.0, 0.0955, 0.213, 0.213);
            std::vector<double> out(r.begin(), r.end());
            return out;
        });
    }

    // local_positions
    {
        Eigen::MatrixXd lp(3, 4);
        lp << 0.2, 0.2, -0.2, -0.2, -0.1, 0.1, -0.1, 0.1, 0, 0, 0, 0;
        bench("local_positions", [&]{
            auto r = quadropted::compute_local_positions(lp, 0.3762, 0.0935, 0.01, 0, 0, 0, 0, 0);
            return vd({r(0,0), r(0,1), r(0,2)});
        });
    }

    // Gait phase_ticks
    {
        Eigen::MatrixXi cp(4, 4);
        cp << 1,1,1,0, 1,0,1,1, 1,0,1,1, 1,1,1,0;
        quadropted::GaitController gc(0.04, 0.18, 0.02, cp, Eigen::MatrixXd::Zero(3, 4));
        bench("GaitController.phase_ticks", [&]{
            const auto& pt = gc.phase_ticks();
            return vd({(double)pt[0], (double)pt[1], (double)pt[2], (double)pt[3]});
        });
    }

    // update_odometry
    {
        bench("update_odometry", []{
            quadropted::OdometryState s;
            s.linear_velocity_x = 0.02; s.linear_velocity_y = 0.01; s.theta = 0.1;
            s.foot_contacts = {false, false, false, false};
            quadropted::update_odometry(s, 0.02);
            return vd({s.x, s.y, s.theta});
        });
    }

    return 0;
}
'''


def build_cpp_benchmark():
    """Скомпилировать C++ benchmark."""
    src_path = os.path.join(BUILD_DIR, "benchmark_cpp.cpp")
    exe_path = os.path.join(BUILD_DIR, "benchmark_cpp")

    with open(src_path, 'w') as f:
        f.write(CPP_BENCH_SOURCE)

    include_dir = os.path.abspath(os.path.join(TESTS_DIR, "..", "quadropted_controller_cpp", "include"))
    eigen_include = subprocess.run(
        ["pkg-config", "--cflags-only-I", "eigen3"],
        capture_output=True, text=True
    ).stdout.strip()
    if not eigen_include:
        eigen_include = "-I/usr/include/eigen3"

    # Ищем библиотеку
    lib_path = os.path.join(BUILD_DIR, "libquadropted_controller_cpp.so")
    if not os.path.exists(lib_path):
        lib_path = os.path.join(BUILD_DIR, "..", "..", "install", "quadropted_controller_cpp", "lib", "libquadropted_controller_cpp.so")

    cmd = [
        "g++", "-std=c++17", "-O2",
        "-I", include_dir,
        eigen_include,
        src_path, "-o", exe_path,
    ]
    if os.path.exists(lib_path):
        lib_dir = os.path.dirname(lib_path)
        cmd.extend(["-L", lib_dir, "-lquadropted_controller_cpp",
                    "-Wl,-rpath," + lib_dir])

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"  ❌ Ошибка компиляции: {result.stderr[:300]}")
        return False
    return True


# ════════════════════════════════════════════════════════════
# Runner
# ════════════════════════════════════════════════════════════

FUNCTIONS = [
    "rotxyz",
    "homog_transform_inverse",
    "FK",
    "IK",
    "local_positions",
    "GaitController.phase_ticks",
    "update_odometry",
]


def main():
    print("=" * 75)
    print(f"C++ vs Python Benchmark ({ITERATIONS} итераций)")
    print("=" * 75)

    # Python
    print("\nЗапуск Python benchmark...", end=" ", flush=True)
    py_results = benchmark_python()
    if py_results is None:
        print("❌ ошибка")
        return 1
    print(f"✅ ({len(py_results)} функций)")

    # C++
    print("Сборка C++ benchmark...", end=" ", flush=True)
    if not build_cpp_benchmark():
        print("❌")
        return 1
    print("✅")

    print("Запуск C++ benchmark...", end=" ", flush=True)
    bench_exe = os.path.join(BUILD_DIR, "benchmark_cpp")
    result = subprocess.run([bench_exe, str(ITERATIONS)], capture_output=True, text=True, timeout=60)
    if result.returncode != 0:
        print(f"❌ {result.stderr[:200]}")
        return 1
    cpp_results = {}
    for line in result.stdout.strip().split('\n'):
        if ':' in line:
            key, val = line.strip().rstrip(',').split(':', 1)
            cpp_results[key.strip('"')] = float(val)
    print(f"✅ ({len(cpp_results)} функций)")

    # Сравнение
    print()
    print(f"{'Функция':<35} {'Python (мс)':>12} {'C++ (мс)':>12} {'Ускорение':>12}")
    print("-" * 75)

    total_py = total_cpp = 0
    for func in FUNCTIONS:
        py_ms = py_results.get(func, 0)
        cpp_ms = cpp_results.get(func, 0)
        total_py += py_ms
        total_cpp += cpp_ms

        if cpp_ms > 0:
            speedup = py_ms / cpp_ms
            print(f"{func:<35} {py_ms:>12.4f} {cpp_ms:>12.4f} {speedup:>10.1f}x")
        else:
            print(f"{func:<35} {py_ms:>12.4f} {'—':>12} {'—':>12}")

    print("-" * 75)
    if total_cpp > 0:
        total_speedup = total_py / total_cpp
        print(f"{'ИТОГО':<35} {total_py:>12.4f} {total_cpp:>12.4f} {total_speedup:>10.1f}x")
    print("=" * 75)

    return 0


if __name__ == "__main__":
    sys.exit(main())
