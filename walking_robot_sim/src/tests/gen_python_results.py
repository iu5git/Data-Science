#!/usr/bin/env python3
"""Генератор Python результатов для сравнения с C++."""
import os, sys, json, math, numpy as np

PYTHON_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "quadropted_controller", "scripts"))
sys.path.insert(0, PYTHON_SCRIPTS)

from ForwardKinematics.forward_kinematics import ForwardKinematics
from RoboticsUtilities.rotation_matrices import rotxyz
from RoboticsUtilities.homogeneous_transforms import homog_transform, homog_transform_inverse
from InverseKinematics.local_positions import compute_local_positions
from InverseKinematics.joint_angles import compute_all_joint_angles
from QuadrupedOdometry.odometry_state import OdometryState
from QuadrupedOdometry.odometry_update import update_odometry, normalize_angle
from QuadrupedOdometry.message_builders import build_quaternion_from_yaw, build_odometry_data, build_tf_data

# Импортируем напрямую, минуя __init__.py RobotController
import importlib.util
def _load_module(path):
    spec = importlib.util.spec_from_file_location("mod", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

gait_path = os.path.join(PYTHON_SCRIPTS, "RobotController", "GaitController.py")
pid_path = os.path.join(PYTHON_SCRIPTS, "RobotController", "PIDController.py")
GaitController = _load_module(gait_path).GaitController
PID_controller = _load_module(pid_path).PID_controller

def to_python(obj):
    if hasattr(obj, 'tolist'): return obj.tolist()
    if isinstance(obj, dict): return {k:to_python(v) for k,v in obj.items()}
    if isinstance(obj, (list, tuple)): return [to_python(x) for x in obj]
    if isinstance(obj, (np.integer,)): return int(obj)
    if isinstance(obj, (np.floating,)): return float(obj)
    return obj

r = {}
r["FK"] = ForwardKinematics([0.3762,0.0935],[0.0,0.0955,0.213,0.213]).forward_kinematics_all_legs([0,0.3,-0.6]*4)
r["IK"] = compute_all_joint_angles([[0.2,-0.12,-0.2],[0.2,0.12,-0.2],[-0.2,-0.12,-0.2],[-0.2,0.12,-0.2]], 0.0, 0.0955, 0.213, 0.213)
lp = np.array([[0.2,0.2,-0.2,-0.2],[-0.1,0.1,-0.1,0.1],[0,0,0,0]])
r["LP"] = compute_local_positions(lp, 0.3762, 0.0935, 0.01, 0,0,0,0,0).tolist()
m = homog_transform(0.1,0.2,0.3,0.4,0.5,0.6)
r["HT_inv"] = (m @ homog_transform_inverse(m.copy())).tolist()
r["rotxyz"] = rotxyz(0.1, -0.05, 0.02).tolist()

state = OdometryState()
state.linear_velocity_x = 0.02; state.linear_velocity_y = 0.01; state.theta = 0.1
state.foot_contacts = [False]*4
update_odometry(state, 0.02)
r["odometry"] = [state.x, state.y, state.theta]

r["phase_ticks"] = GaitController(0.04, 0.18, 0.02, np.array([[1,1,1,0],[1,0,1,1],[1,0,1,1],[1,1,1,0]]), np.zeros((3,4))).phase_ticks

pid = PID_controller(0.15, 0.02, 0.002)
pid.last_time = pid.get_time_in_seconds()
r["pid"] = list(pid.run(0.1, -0.05))

r["normalize"] = [normalize_angle(a) for a in [0, math.pi, -math.pi, 3*math.pi, -3*math.pi]]
r["quat"] = [list(build_quaternion_from_yaw(a)) for a in [0, 0.5, 1.0, -0.5]]

d = build_odometry_data(1.0, 2.0, 0.5, 0.1, 0.05, 0.02, 'odom', 'base', 'now')
r["odom"] = [d['pose_position'][0], d['pose_position'][1], d['twist_linear'][0], d['twist_linear'][1], d['twist_angular'][2]]

d2 = build_tf_data(1.0, 2.0, 0.5, 'odom', 'base', 'now')
r["tf"] = [d2['translation'][0], d2['translation'][1], d2['rotation'][2]]

print(json.dumps(to_python(r)))
