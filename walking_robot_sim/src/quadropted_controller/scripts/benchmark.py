#!/usr/bin/env python3
"""
Python Quadruped Controller Benchmark
Сравнение с C++ версией - использует существующие классы из проекта
Запускать внутри Docker контейнера с ROS2
"""

import os
import sys
import time

# Add source path
sys.path.insert(0, "/root/ws/src/quadropted_controller/scripts")

import numpy as np


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


def step_trot(tick, current_foot_locations, state):
    contacts = gait.contacts(tick)
    result = current_foot_locations.copy()
    for leg in range(4):
        if contacts[leg] == 1:
            result[:, leg] = stance_ctrl.next_foot_location(leg, state, command)
        else:
            sub = gait.subphase_ticks(tick)
            swing_prop = sub / 9.0
            result[:, leg] = swing.next_foot_location(swing_prop, leg, state, command)
    return result


start = time.perf_counter()
current = stance.copy()
for i in range(iterations):
    current = step_trot(i % 22, current, state)
end = time.perf_counter()
duration = (end - start) * 1e6
print(
    f"  Trot step: {iterations} iterations in {duration:.0f} μs (~{duration / iterations:.2f} μs/call)"
)

start = time.perf_counter()
for i in range(iterations):
    joints = ik.inverse_kinematics(stance, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0)
end = time.perf_counter()
duration = (end - start) * 1e6
print(
    f"  InverseKinematics: {iterations} iterations in {duration:.0f} μs (~{duration / iterations:.2f} μs/call)"
)

print("\n" + "=" * 60)
print("Benchmark completed successfully!")
print("=" * 60)
