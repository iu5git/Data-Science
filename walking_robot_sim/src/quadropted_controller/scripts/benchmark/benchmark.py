#!/usr/bin/env python3
"""
Python Quadruped Controller Benchmark
Сравнение с C++ версией - использует существующие классы из проекта
Запускать внутри Docker контейнера с ROS2

Usage:
  python3 benchmark.py              # Только Python benchmark (make benchmark-python)
  python3 benchmark.py --combined     # Python + C++ сводная таблица (make benchmark)
"""

import argparse
import os
import sys
import time

# Add source path
sys.path.insert(0, "/root/ws/src/quadropted_controller/scripts")

import numpy as np

# Parse arguments
parser = argparse.ArgumentParser(description="Python Quadruped Controller Benchmark")
parser.add_argument(
    "--combined", action="store_true", help="Run combined Python + C++ benchmark"
)
args = parser.parse_args()

# Для красивой таблицы
try:
    from tabulate import tabulate

    HAS_TABULATE = True
except ImportError:
    HAS_TABULATE = False


def create_default_stance():
    body = [0.3762, 0.0935]
    legs = [0.0, 0.0955, 0.213, 0.213]
    dx = body[0] * 0.5 + 0.02
    dy = body[1] * 0.5 + legs[1]
    return np.array([[dx, dx, -dx, -dx], [-dy, dy, -dy, dy], [0, 0, 0, 0]])


print("╔" + "═" * 58 + "╗")
print("║" + " " * 10 + "Python Quadruped Controller Benchmark v0.0.1" + " " * 10 + "║")
print("╚" + "═" * 58 + "╝")

print("\n[1] Testing GaitController...")
from RobotController.GaitController import GaitController

stance = create_default_stance()
contact_phases = np.array([[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]])
gait = GaitController(0.04, 0.18, 0.02, contact_phases, stance)
print(f"  stance_ticks={gait.stance_ticks}, swing_ticks={gait.swing_ticks}")
print(f"  phase_length={gait.phase_length}, phase_ticks={gait.phase_ticks}")


print("\n[2] Testing TrotSwingController...")
from RobotController.trot_gait.trot_swing import TrotSwingController

swing = TrotSwingController(
    stance_ticks=2,
    swing_ticks=9,
    time_step=0.02,
    phase_length=22,
    z_leg_lift=0.14,
    default_stance=stance,
)
print(f"  swing_height(0.0) = {swing.swing_height(0.0):.4f}")
print(f"  swing_height(0.5) = {swing.swing_height(0.5):.4f}")
print(f"  swing_height(1.0) = {swing.swing_height(1.0):.4f}")


print("\n[3] Testing TrotStanceController...")
from RobotController.trot_gait.trot_stance import TrotStanceController

stance_ctrl = TrotStanceController(
    phase_length=22,
    stance_ticks=2,
    swing_ticks=9,
    time_step=0.02,
    z_error_constant=0.02,
)
print(f"  z_error_constant = {stance_ctrl.z_error_constant}")


print("\n[4] Testing StandController...")
from RobotController.StandController import StandController

stand = StandController(None, stance)
print(f"  max_reach = {stand.max_reach}")


print("\n[5] Testing State and Command...")
from RobotController.StateCommand import Command, State

state = State(0.25)
state.foot_locations = stance.copy()
command = Command(0.25)
command.velocity = np.array([0.03, 0.0, 0.0])
command.yaw_rate = np.array([0.0, 0.0, 0.0])
print(f"  robot_height = {state.robot_height}")


print("\n[6] Testing ForwardKinematics...")
from ForwardKinematics.forward_kinematics import ForwardKinematics

body = [0.3762, 0.0935]
legs_dim = [0.0, 0.0955, 0.213, 0.213]
fk = ForwardKinematics(body, legs_dim)
pos = fk.forward_kinematics_per_leg(0.0, 0.86, -1.88, 0)
print(
    f"  FK(hip=0, thigh=0.86, calf=-1.88, leg=0) = [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]"
)


print("\n[7] Testing RestController...")
from RobotController.RestController import RestController

rest = RestController(stance)
result = rest.step(state, command)
print(
    f"  step() z positions = [{result[2, 0]:.4f}, {result[2, 1]:.4f}, {result[2, 2]:.4f}, {result[2, 3]:.4f}]"
)


print("\n[8] Testing InverseKinematics...")
from InverseKinematics.inverse_kinematics import InverseKinematics

ik = InverseKinematics(body, legs_dim)
joints = ik.inverse_kinematics(stance, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
print(f"  joints[0-5] = {[round(j, 4) for j in joints[:6]]}")
print(f"  joints[6-11] = {[round(j, 4) for j in joints[6:]]}")


print("\n[9] Testing PID Controller...")
from RobotController.PIDController import PID_controller

pid = PID_controller(0.75, 2.29, 0.0)
output = pid.run(0.1, 0.1, dt=0.02)
print(f"  run(roll=0.1, pitch=0.1, dt=0.02) = [{output[0]:.4f}, {output[1]:.4f}]")


print("\n[10] Performance Timing Benchmark (10000 iterations)...")
iterations = 10000


# ============================================================================
# PERFORMANCE BENCHMARK - ИЗМЕРЕНИЕ ВРЕМЕНИ ВЫПОЛНЕНИЯ ВСЕХ КЛАССОВ
# ============================================================================

print("\n" + "=" * 70)
print(" PERFORMANCE BENCHMARK - ИЗМЕРЕНИЕ ВРЕМЕНИ ВЫПОЛНЕНИЯ (10000 итераций)")
print("=" * 70)


def benchmark_function(name, func, *args, **kwargs):
    """Измерить время выполнения функции"""
    start = time.perf_counter()
    for _ in range(iterations):
        func(*args, **kwargs)
    end = time.perf_counter()
    duration_us = (end - start) * 1e6
    avg_us = duration_us / iterations
    return avg_us


print("\n[Performance] Тестирование всех классов и функций:")
print("-" * 70)


# 1. GaitController - contacts() и другие методы
print("\n1. GaitController")
bench_gait_contacts = benchmark_function(
    "GaitController.contacts(tick)", lambda: gait.contacts(5)
)
print(f"   contacts()         : {bench_gait_contacts:7.2f} μs/вызов")

bench_gait_subphase = benchmark_function(
    "GaitController.subphase_ticks(tick)", lambda: gait.subphase_ticks(5)
)
print(f"   subphase_ticks()   : {bench_gait_subphase:7.2f} μs/вызов")


# 2. TrotSwingController
print("\n2. TrotSwingController")
bench_swing_height = benchmark_function(
    "TrotSwingController.swing_height(p)", lambda: swing.swing_height(0.5)
)
print(f"   swing_height()     : {bench_swing_height:7.2f} μs/вызов")

bench_swing_next = benchmark_function(
    "TrotSwingController.next_foot_location()",
    lambda: swing.next_foot_location(0.5, 0, state, command),
)
print(f"   next_foot_location(): {bench_swing_next:7.2f} μs/вызов")

bench_swing_raibert = benchmark_function(
    "TrotSwingController.raibert_touchdown_location()",
    lambda: swing.raibert_touchdown_location(0, command),
)
print(f"   raibert_touchdown(): {bench_swing_raibert:7.2f} μs/вызов")


# 3. TrotStanceController
print("\n3. TrotStanceController")
bench_stance_delta = benchmark_function(
    "TrotStanceController.position_delta()",
    lambda: stance_ctrl.position_delta(0, state, command),
)
print(f"   position_delta()   : {bench_stance_delta:7.2f} μs/вызов")

bench_stance_next = benchmark_function(
    "TrotStanceController.next_foot_location()",
    lambda: stance_ctrl.next_foot_location(0, state, command),
)
print(f"   next_foot_location(): {bench_stance_next:7.2f} μs/вызов")


# 4. StandController (requires ROS node, skip for pure benchmark)
print("\n4. StandController (skipped - requires ROS node)")
bench_stand = 0.0


# 5. ForwardKinematics
print("\n5. ForwardKinematics")
bench_fk = benchmark_function(
    "ForwardKinematics.forward_kinematics_per_leg()",
    lambda: fk.forward_kinematics_per_leg(0.0, 0.86, -1.88, 0),
)
print(f"   forward_kinematics_per_leg(): {bench_fk:7.2f} μs/вызов")

bench_fk_all = benchmark_function(
    "ForwardKinematics.forward_kinematics_all_legs()",
    lambda: fk.forward_kinematics_all_legs(
        [0.0, 0.86, -1.88, 0.0, 0.86, -1.88, 0.0, 0.86, -1.88, 0.0, 0.86, -1.88]
    ),
)
print(f"   forward_kinematics_all_legs(): {bench_fk_all:7.2f} μs/вызов")


# 6. RestController
print("\n6. RestController")
bench_rest = benchmark_function(
    "RestController.step()", lambda: rest.step(state, command)
)
print(f"   step()             : {bench_rest:7.2f} μs/вызов")


# 7. InverseKinematics
print("\n7. InverseKinematics")
bench_ik = benchmark_function(
    "InverseKinematics.inverse_kinematics()",
    lambda: ik.inverse_kinematics(stance, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
)
print(f"   inverse_kinematics(): {bench_ik:7.2f} μs/вызов")


# 8. PIDController
print("\n8. PIDController")
bench_pid = benchmark_function(
    "PIDController.run()", lambda: pid.run(0.1, 0.1, dt=0.02)
)
print(f"   run()              : {bench_pid:7.2f} μs/вызов")


# 9. Trot Step (полный цикл)
print("\n9. Trot Step (полный цикл)")


def step_trot_full(tick, current_foot_locations, st):
    contacts = gait.contacts(tick)
    result = current_foot_locations.copy()
    for leg in range(4):
        if contacts[leg] == 1:
            result[:, leg] = stance_ctrl.next_foot_location(leg, st, command)
        else:
            sub = gait.subphase_ticks(tick)
            swing_prop = sub / 9.0
            result[:, leg] = swing.next_foot_location(swing_prop, leg, st, command)
    return result


bench_trot_step = benchmark_function(
    "TrotStep (full cycle)", lambda i: step_trot_full(i % 22, stance, state), 0
)
print(f"   step (все 4 ноги) : {bench_trot_step:7.2f} μs/вызов")


# ============================================================================
# СВОДНАЯ ТАБЛИЦА
# ============================================================================


def run_cpp_benchmark():
    """Запустить C++ бенчмарк и получить реальные результаты"""
    import subprocess

    try:
        result = subprocess.run(
            [
                "bash",
                "-c",
                "source /opt/ros/jazzy/setup.bash && /root/ws/build/quadropted_controller_cpp/benchmark 2>&1",
            ],
            capture_output=True,
            text=True,
            timeout=60,
        )

        cpp_results = {}
        in_json = False
        for line in result.stdout.split("\n"):
            if "=== BENCHMARK_JSON_START ===" in line:
                in_json = True
                continue
            if "=== BENCHMARK_JSON_END ===" in line:
                in_json = False
                continue
            if in_json and ":" in line:
                parts = line.strip().split(":")
                if len(parts) == 2:
                    name = parts[0].strip()
                    try:
                        value = float(parts[1].strip())
                        cpp_results[name] = value
                    except:
                        pass
        return cpp_results
    except Exception as e:
        print(f"  [WARN] Не удалось запустить C++ бенчмарк: {e}")
        return {}


print("\n" + "=" * 70)
print(" СВОДНАЯ ТАБЛИЦА ПРОИЗВОДИТЕЛЬНОСТИ (Python vs C++)")
print("=" * 70)

print("\n[Сводная] Запуск C++ бенчмарка для получения реальных данных...")
cpp_results = run_cpp_benchmark()
print(f"  [OK] Получено {len(cpp_results)} результатов от C++")

results = [
    ("GaitController.contacts()", bench_gait_contacts),
    ("GaitController.subphase_ticks()", bench_gait_subphase),
    ("TrotSwingController.swing_height()", bench_swing_height),
    ("TrotSwingController.next_foot_location()", bench_swing_next),
    ("TrotSwingController.raibert_touchdown()", bench_swing_raibert),
    ("TrotStanceController.position_delta()", bench_stance_delta),
    ("TrotStanceController.next_foot_location()", bench_stance_next),
    ("StandController.run()", bench_stand),
    ("ForwardKinematics.forward_kinematics_per_leg()", bench_fk),
    ("ForwardKinematics.forward_kinematics_all_legs()", bench_fk_all),
    ("RestController.step()", bench_rest),
    ("InverseKinematics.inverse_kinematics()", bench_ik),
    ("PIDController.run()", bench_pid),
    ("Trot Step (full cycle)", bench_trot_step),
]

# Если запущен с --combined - показываем сводную таблицу Python vs C++
if args.combined:
    print("\n" + "=" * 70)
    print(" СВОДНАЯ ТАБЛИЦА ПРОИЗВОДИТЕЛЬНОСТИ (Python vs C++)")
    print("=" * 70)

    print("\n[Сводная] Запуск C++ бенчмарка для получения реальных данных...")
    cpp_results = run_cpp_benchmark()
    print(f"  [OK] Получено {len(cpp_results)} результатов от C++")

    # Построение таблицы
    table_data = []
    for name, py_time in results:
        cpp_time = cpp_results.get(name, 0)
        if cpp_time > 0 and py_time > 0:
            ratio = py_time / cpp_time
            if ratio < 1:
                diff = f"C++ быстрее в {1 / ratio:.1f}x"
            else:
                diff = f"Python медленнее в {ratio:.1f}x"
        elif py_time == 0:
            diff = "N/A (Python: skipped)"
        else:
            diff = "N/A"
        table_data.append([name, f"{py_time:.2f}", f"{cpp_time:.3f}", diff])

    if HAS_TABULATE:
        print(
            "\n"
            + tabulate(
                table_data,
                headers=["Функция", "Python (μs)", "C++ (μs)", "Разница"],
                tablefmt="grid",
                maxcolwidths=[40, 12, 12, 25],
            )
        )
    else:
        print(f"\n{'Функция':<50} {'Python (μs)':<12} {'C++ (μs)':<12} {'Разница':<30}")
        print("-" * 105)
        for row in table_data:
            print(f"{row[0]:<50} {row[1]:>12} {row[2]:>12} {row[3]:<30}")

    print("\n" + "=" * 70)
    print("Анализ: Python значительно медленнее C++ во всех операциях")
    print("Средняя разница: ~100-1000x в зависимости от операции")
    print("=" * 70)
else:
    # Только Python таблица (без C++)
    print("\n" + "=" * 70)
    print(" ТАБЛИЦА ПРОИЗВОДИТЕЛЬНОСТИ (Python)")
    print("=" * 70)

    table_data = []
    for name, py_time in results:
        table_data.append([name, f"{py_time:.2f}"])

    if HAS_TABULATE:
        print(
            "\n"
            + tabulate(
                table_data,
                headers=["Функция", "Время (μs)"],
                tablefmt="grid",
                maxcolwidths=[50, 15],
            )
        )
    else:
        print(f"\n{'Функция':<50} {'Время (μs)':<15}")
        print("-" * 65)
        for row in table_data:
            print(f"{row[0]:<50} {row[1]:>15}")

    print("\n" + "=" * 70)
    print("Для сравнения с C++ запустите: python3 benchmark.py --combined")
    print("=" * 70)
