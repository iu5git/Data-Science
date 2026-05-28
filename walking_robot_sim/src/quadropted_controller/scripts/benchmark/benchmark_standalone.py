#!/usr/bin/env python3
"""
Python Quadruped Controller Benchmark (standalone - no ROS dependencies)
Сравнение с C++ версией - только математические операции
"""

import time

import numpy as np


class GaitController:
    def __init__(
        self, stance_time, swing_time, time_step, contact_phases, default_stance
    ):
        self.stance_time = stance_time
        self.swing_time = swing_time
        self.time_step = time_step
        self.contact_phases = contact_phases
        self.def_stance = default_stance

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
        return len(self.contact_phases[0]) - 1

    def subphase_ticks(self, ticks):
        phase_time = ticks % self.phase_length
        phase_sum = 0
        phase_ticks = self.phase_ticks
        for i in range(len(self.contact_phases[0])):
            if phase_time < phase_sum + phase_ticks[i]:
                return phase_time - phase_sum
            phase_sum += phase_ticks[i]
        return 0

    def contacts(self, ticks):
        phase_idx = self.phase_index(ticks)
        return self.contact_phases[phase_idx]


class TrotSwingController:
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

    def swing_height(self, swing_prop):
        return self.z_leg_lift * np.sin(swing_prop * np.pi)

    def raibert_touchdown_location(self, leg, command):
        vx = command.velocity[0]
        touchdown_x = vx * (self.swing_ticks * self.time_step) / 2.0
        return np.array([touchdown_x, 0.0, 0.0])

    def next_foot_location(self, swing_prop, leg, state, command):
        default = self.default_stance[:, leg]
        touch = self.raibert_touchdown_location(leg, command)
        height = self.swing_height(swing_prop)

        result = default.copy()
        result[0] += touch[0]
        result[2] = -state.robot_height + height

        return result


class TrotStanceController:
    def __init__(
        self, phase_length, stance_ticks, swing_ticks, time_step, z_error_constant
    ):
        self.phase_length = phase_length
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.z_error_constant = z_error_constant

    def position_delta(self, leg, state, command):
        robot_height = state.robot_height
        vx = command.velocity[0]
        vy = command.velocity[1]
        wz = command.yaw_rate[2]

        dx = vx * self.time_step
        dy = vy * self.time_step
        dz = -self.z_error_constant * (robot_height - state.foot_locations[2, leg])

        return np.array([dx, dy, dz])

    def next_foot_location(self, leg, state, command):
        current = state.foot_locations[:, leg].copy()
        delta = self.position_delta(leg, state, command)
        return current + delta


class State:
    def __init__(self, robot_height):
        self.robot_height = robot_height
        self.foot_locations = None


class Command:
    def __init__(self, robot_height):
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.yaw_rate = np.array([0.0, 0.0, 0.0])


class InverseKinematics:
    def __init__(self, body, legs):
        self.hip_x = body[0] / 2.0 + 0.02
        self.hip_y = body[1] / 2.0 + legs[1]
        self.upper_leg = legs[2]
        self.lower_leg = legs[3]

    def inverse_kinematics(self, foot_locations, dx, dy, dz, roll, pitch, yaw):
        joints = np.zeros(12)
        for leg in range(4):
            foot = foot_locations[:, leg]
            hip_x = ((-1) ** leg) * self.hip_x
            hip_y = ((-1) ** (leg + 1)) * self.hip_y

            x = foot[0] + dx + hip_x
            y = foot[1] + dy
            z = foot[2] - dz

            dist = np.sqrt(x * x + y * y + z * z)
            cos_angle = (self.upper_leg**2 + self.lower_leg**2 - dist**2) / (
                2 * self.upper_leg * self.lower_leg
            )
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            calf_angle = np.pi - np.arccos(cos_angle)

            cos_lower = (dist**2 + self.upper_leg**2 - self.lower_leg**2) / (
                2 * dist * self.upper_leg
            )
            cos_lower = np.clip(cos_lower, -1.0, 1.0)
            lower_angle = np.arccos(cos_lower)

            hip_angle = np.arctan2(y, np.sqrt(x * x + z * z))
            thigh_angle = lower_angle
            calf_angle = np.pi - calf_angle

            joints[leg * 3] = hip_angle
            joints[leg * 3 + 1] = thigh_angle
            joints[leg * 3 + 2] = -calf_angle

        return joints


class ForwardKinematics:
    def __init__(self, body, legs):
        self.hip_x = body[0] / 2.0 + 0.02
        self.hip_y = body[1] / 2.0 + legs[1]
        self.upper_leg = legs[2]
        self.lower_leg = legs[3]

    def forward_kinematics(self, hip_angles, leg):
        hip = hip_angles[0]
        thigh = hip_angles[1]
        calf = hip_angles[2]

        sign_x = (-1) ** leg
        sign_y = (-1) ** (leg + 1)

        hip_x = sign_x * self.hip_x
        hip_y = sign_y * self.hip_y

        x1 = self.upper_leg * np.sin(thigh)
        z1 = self.upper_leg * np.cos(thigh)

        x2 = self.lower_leg * np.sin(thigh + calf)
        z2 = self.lower_leg * np.cos(thigh + calf)

        x = hip_x + x1 + x2
        y = hip_y
        z = z1 + z2

        return np.array([x, y, z])


class RestController:
    def __init__(self, default_stance):
        self.default_stance = default_stance.copy()
        self.kp = 0.75
        self.ki = 2.29
        self.kd = 0.0

    def step(self, state, command):
        result = self.default_stance.copy()
        for leg in range(4):
            result[2, leg] = -state.robot_height
        return result


class StandController:
    def __init__(self, default_stance):
        self.default_stance = default_stance.copy()
        self.max_reach = 0.065
        self.body_velocity_scale = 0.01
        self.body_angular_scale = 0.005

    def run(self, state, command):
        return self.default_stance.copy()


class PID_controller:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.desired_roll_pitch = np.array([0.0, 0.0])
        self.I_term = np.array([0.0, 0.0])
        self.D_term = np.array([0.0, 0.0])
        self.max_I = 0.2
        self.last_error = np.array([0.0, 0.0])
        self.last_time = 0.0

    def run(self, roll, pitch, dt=None):
        error = self.desired_roll_pitch - np.array([roll, pitch])

        if dt is None:
            dt = 0.02

        if dt < 1e-6:
            return np.array([0.0, 0.0])

        self.I_term += error * dt

        for i in range(2):
            if self.I_term[i] < -self.max_I:
                self.I_term[i] = -self.max_I
            elif self.I_term[i] > self.max_I:
                self.I_term[i] = self.max_I

        self.D_term = (error - self.last_error) / dt
        self.last_time += dt
        self.last_error = error

        P_ret = self.kp * error
        I_ret = self.I_term * self.ki
        D_ret = self.D_term * self.kd

        return P_ret + I_ret + D_ret

    def set_desired_RP_angles(self, des_roll, des_pitch):
        self.desired_roll_pitch = np.array([des_roll, des_pitch])


def create_default_stance():
    body = [0.3762, 0.0935]
    legs = [0.0, 0.0955, 0.213, 0.213]
    dx = body[0] * 0.5 + 0.02
    dy = body[1] * 0.5 + legs[1]
    return np.array([[dx, dx, -dx, -dx], [-dy, dy, -dy, dy], [0, 0, 0, 0]])


def benchmark_gait_controller():
    print("\n" + "=" * 60)
    print("Gait Controller Benchmark")
    print("=" * 60)

    stance = create_default_stance()
    contact_phases = np.array([[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]])

    gait = GaitController(0.04, 0.18, 0.02, contact_phases, stance)

    print("Parameters:")
    print("  stance_time: 0.04, swing_time: 0.18, time_step: 0.02")
    print(f"  stance_ticks: {gait.stance_ticks}")
    print(f"  swing_ticks: {gait.swing_ticks}")
    print(f"  phase_length: {gait.phase_length}")
    print(f"  phase_ticks: {gait.phase_ticks}")

    print("\nPhase contacts at ticks:")
    for tick in range(0, 23, 2):
        c = gait.contacts(tick)
        print(f"  tick={tick}: {c.tolist()}")


def benchmark_trot_swing():
    print("\n" + "=" * 60)
    print("TrotSwing Controller Benchmark")
    print("=" * 60)

    stance = create_default_stance()
    swing = TrotSwingController(
        stance_ticks=2,
        swing_ticks=9,
        time_step=0.02,
        phase_length=22,
        z_leg_lift=0.14,
        default_stance=stance,
    )

    print("Parameters: swing_ticks=9, time_step=0.02, z_leg_lift=0.14")

    print("\nswing_height at different phases:")
    for p in np.arange(0.0, 1.1, 0.1):
        h = swing.swing_height(p)
        print(f"  swing_prop={p:.1f}: height={h:.4f}")

    cmd_vel = np.array([0.03, 0.0, 0.0])
    command = Command(0.25)
    command.velocity = cmd_vel

    print("\nraibert_touchdown_location for leg 0 (vx=0.03):")
    touch = swing.raibert_touchdown_location(0, command)
    print(f"  [{float(touch[0]):.4f}, {float(touch[1]):.4f}, {float(touch[2]):.4f}]")

    print("\nnext_foot_location (swing_prop=0.5, leg=0, robot_height=0.25):")
    state = State(0.25)
    state.foot_locations = stance.copy()
    next_foot = swing.next_foot_location(0.5, 0, state, command)
    print(
        f"  [{float(next_foot[0]):.4f}, {float(next_foot[1]):.4f}, {float(next_foot[2]):.4f}]"
    )


def benchmark_trot_stance():
    print("\n" + "=" * 60)
    print("TrotStance Controller Benchmark")
    print("=" * 60)

    stance = create_default_stance()
    stance_ctrl = TrotStanceController(
        phase_length=22,
        stance_ticks=2,
        swing_ticks=9,
        time_step=0.02,
        z_error_constant=0.02,
    )

    print(
        "Parameters: phase_length=22, stance_ticks=2, swing_ticks=9, z_error_constant=0.02"
    )

    cmd_vel = np.array([0.03, 0.0, 0.0])
    command = Command(0.25)
    command.velocity = cmd_vel
    command.yaw_rate = np.array([0.0, 0.0, 0.0])

    print("\nposition_delta for leg 0 (vx=0.03, robot_height=0.25):")
    state = State(0.25)
    state.foot_locations = stance.copy()
    delta = stance_ctrl.position_delta(0, state, command)
    print(f"  delta = {delta}")

    print("\nnext_foot_location for leg 0:")
    next_foot = stance_ctrl.next_foot_location(0, state, command)
    print(f"  next_foot = {next_foot}")


def benchmark_inverse_kinematics():
    print("\n" + "=" * 60)
    print("Inverse Kinematics Benchmark")
    print("=" * 60)

    body = [0.3762, 0.0935]
    legs = [0.0, 0.0955, 0.213, 0.213]
    ik = InverseKinematics(body, legs)

    stance = create_default_stance()

    print("Testing IK for default stance (standing):")
    joints = ik.inverse_kinematics(stance, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    print(f"  joints[0-11] (all legs): {[round(float(j), 4) for j in joints]}")

    print("\nPer-leg joints (hip, thigh, calf):")
    for leg in range(4):
        print(
            f"  Leg {leg}: [{joints[leg * 3]:.4f}, {joints[leg * 3 + 1]:.4f}, {joints[leg * 3 + 2]:.4f}]"
        )

    print("\nIK with body offset (dx=0.1, dy=0.05, dz=0.25):")
    joints2 = ik.inverse_kinematics(stance, 0.1, 0.05, 0.25, 0.0, 0.0, 0.0)
    print(f"  joints[0-11]: {[round(float(j), 4) for j in joints2]}")


def benchmark_forward_kinematics():
    print("\n" + "=" * 60)
    print("Forward Kinematics Benchmark")
    print("=" * 60)

    body = [0.3762, 0.0935]
    legs = [0.0, 0.0955, 0.213, 0.213]
    fk = ForwardKinematics(body, legs)

    print("Testing FK with zero angles:")
    hip_angles = [0.0, 0.0, 0.0]
    pos = fk.forward_kinematics(hip_angles, 0)
    print(f"  Leg 0: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")

    print("\nFK with typical standing angles (0, 0.86, -1.88):")
    hip_angles = [0.0, 0.86, -1.88]
    pos = fk.forward_kinematics(hip_angles, 0)
    print(f"  Leg 0: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")


def benchmark_rest_controller():
    print("\n" + "=" * 60)
    print("Rest Controller Benchmark")
    print("=" * 60)

    stance = create_default_stance()
    rest = RestController(stance)

    state = State(0.25)
    state.foot_locations = stance.copy()
    command = Command(0.25)

    print("Testing Rest controller step:")
    result = rest.step(state, command)
    print("  Result:")
    print(f"  FR: [{result[0, 0]:.4f}, {result[1, 0]:.4f}, {result[2, 0]:.4f}]")
    print(f"  FL: [{result[0, 1]:.4f}, {result[1, 1]:.4f}, {result[2, 1]:.4f}]")
    print(f"  RR: [{result[0, 2]:.4f}, {result[1, 2]:.4f}, {result[2, 2]:.4f}]")
    print(f"  RL: [{result[0, 3]:.4f}, {result[1, 3]:.4f}, {result[2, 3]:.4f}]")


def benchmark_stand_controller():
    print("\n" + "=" * 60)
    print("Stand Controller Benchmark")
    print("=" * 60)

    stance = create_default_stance()
    stand = StandController(stance)

    state = State(0.25)
    state.foot_locations = stance.copy()
    command = Command(0.25)
    command.velocity = np.array([0.0, 0.0, 0.0])
    command.yaw_rate = np.array([0.0, 0.0, 0.0])

    print("Testing Stand controller run:")
    result = stand.run(state, command)
    print("  Result:")
    print(f"  FR: [{result[0, 0]:.4f}, {result[1, 0]:.4f}, {result[2, 0]:.4f}]")
    print(f"  FL: [{result[0, 1]:.4f}, {result[1, 1]:.4f}, {result[2, 1]:.4f}]")
    print(f"  RR: [{result[0, 2]:.4f}, {result[1, 2]:.4f}, {result[2, 2]:.4f}]")
    print(f"  RL: [{result[0, 3]:.4f}, {result[1, 3]:.4f}, {result[2, 3]:.4f}]")


def benchmark_pid_controller():
    print("\n" + "=" * 60)
    print("PID Controller Benchmark")
    print("=" * 60)

    pid = PID_controller(0.75, 2.29, 0.0)

    print("Parameters: kp=0.75, ki=2.29, kd=0.0")

    print("\nPID response to roll=0.1, pitch=0.1 over 10 steps:")
    for i in range(11):
        output = pid.run(0.1, 0.1, dt=i * 0.02)
        print(f"  step={i}: roll_comp={output[0]:.4f}, pitch_comp={output[1]:.4f}")


def benchmark_timing():
    print("\n" + "=" * 60)
    print("Performance Timing Benchmark")
    print("=" * 60)

    stance = create_default_stance()
    contact_phases = np.array([[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]])

    gait = GaitController(0.04, 0.18, 0.02, contact_phases, stance)
    swing = TrotSwingController(2, 9, 0.02, 22, 0.14, stance)
    stance_ctrl = TrotStanceController(22, 2, 9, 0.02, 0.02)

    body = [0.3762, 0.0935]
    legs = [0.0, 0.0955, 0.213, 0.213]
    ik = InverseKinematics(body, legs)

    cmd_vel = np.array([0.03, 0.0, 0.0])
    robot_height = 0.25

    state = State(robot_height)
    state.foot_locations = stance.copy()
    command = Command(robot_height)
    command.velocity = cmd_vel

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
                result[:, leg] = swing.next_foot_location(
                    swing_prop, leg, state, command
                )
        return result

    start = time.perf_counter()
    current = stance.copy()
    for i in range(iterations):
        current = step_trot(i % 22, current, state)
    end = time.perf_counter()
    duration = (end - start) * 1e6

    print("Trot step (swing + stance):")
    print(f"  {iterations} iterations in {duration:.0f} microseconds")
    print(f"  ~{duration / iterations:.2f} microseconds per call")

    start = time.perf_counter()
    for i in range(iterations):
        joints = ik.inverse_kinematics(stance, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0)
    end = time.perf_counter()
    duration = (end - start) * 1e6

    print("\nInverseKinematics.inverse_kinematics():")
    print(f"  {iterations} iterations in {duration:.0f} microseconds")
    print(f"  ~{duration / iterations:.2f} microseconds per call")


def main():
    print("╔" + "═" * 58 + "╗")
    print(
        "║" + " " * 10 + "Python Quadruped Controller Benchmark v0.0.1" + " " * 10 + "║"
    )
    print("╚" + "═" * 58 + "╝")

    benchmark_gait_controller()
    benchmark_trot_swing()
    benchmark_trot_stance()
    benchmark_rest_controller()
    benchmark_stand_controller()
    benchmark_pid_controller()
    benchmark_inverse_kinematics()
    benchmark_forward_kinematics()
    benchmark_timing()

    print("\n" + "=" * 60)
    print("Benchmark completed successfully!")
    print("=" * 60)


if __name__ == "__main__":
    main()
