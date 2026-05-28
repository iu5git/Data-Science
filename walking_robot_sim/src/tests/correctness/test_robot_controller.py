#!/usr/bin/env python3
"""
Comprehensive тест для RobotController: TrotGaitController, CrawlGaitController, PIDController и т.д.
"""

import subprocess
import sys
import os
import tempfile
import shutil
import stat
import importlib.util
import re

BASE_DIR = "/home/redalexdad/GitHub/WalkingRobotSim"
NEW_DIR = BASE_DIR + "/src/quadropted_controller/scripts"
OLD_DIR = BASE_DIR + "/src/tests/old"

test_dir = tempfile.mkdtemp()
modules_dir = os.path.join(test_dir, "modules")
os.makedirs(modules_dir)

for subdir in [
    "ForwardKinematics",
    "InverseKinematics",
    "RoboticsUtilities",
    "QuadrupedOdometry",
    "RobotController",
]:
    src = os.path.join(NEW_DIR, subdir)
    dst = os.path.join(modules_dir, subdir)
    if os.path.exists(src):
        shutil.copytree(src, dst)
        with open(os.path.join(dst, "__init__.py"), "w") as f:
            pass

shutil.copy(
    OLD_DIR + "/RoboticsUtilities/Transformations.py",
    modules_dir + "/RoboticsUtilities/Transformations.py",
)

for root, dirs, files in os.walk(modules_dir):
    rel_path = os.path.relpath(root, modules_dir)
    if rel_path == ".":
        continue
    parent_pkg = rel_path.replace(os.sep, ".")

    for f in files:
        if f.endswith(".py") and f != "Transformations.py":
            fp = os.path.join(root, f)
            with open(fp, "r") as file:
                content = file.read()

            new_content = re.sub(
                r"^from \.(\w+)", f"from {parent_pkg}.\\1", content, flags=re.MULTILINE
            )

            if new_content != content:
                with open(fp, "w") as file:
                    file.write(new_content)

final_path = "/tmp/test_robot_controller"
if os.path.exists(final_path):
    shutil.rmtree(final_path)
shutil.copytree(modules_dir, final_path)

test_script_path = os.path.join(test_dir, "run_robot_tests.py")

test_code = r"""#!/usr/bin/env python3
import sys
import os
import math
import numpy as np
import importlib.util

def load_module_from_path(module_name, filepath):
    spec = importlib.util.spec_from_file_location(module_name, filepath)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module

sys.path.insert(0, "/tmp/test_robot_controller/RoboticsUtilities")
sys.path.insert(0, "/tmp/test_robot_controller")

# Import new modules
from RobotController.trot_gait.trot_stance import TrotStanceController as TS_new
from RobotController.trot_gait.trot_swing import TrotSwingController as TSw_new
from RobotController.trot_gait.trot_gait import TrotGaitController as TG_new
from RobotController.crawl_gait.crawl_stance import CrawlStanceController as CS_new
from RobotController.crawl_gait.crawl_swing import CrawlSwingController as CSw_new
from RobotController.crawl_gait.crawl_gait import CrawlGaitController as CG_new
from RobotController.PIDController import PID_controller as PID_new
from RobotController.GaitController import GaitController as GC_new

OLD_DIR = "/home/redalexdad/GitHub/WalkingRobotSim/src/tests/old"

# Load old modules
TrotGait_old_mod = load_module_from_path("old_TrotGait", OLD_DIR + "/RobotController/TrotGaitController.py")
TrotStance_old = TrotGait_old_mod.TrotStanceController
TrotSwing_old = TrotGait_old_mod.TrotSwingController

CrawlGait_old_mod = load_module_from_path("old_CrawlGait", OLD_DIR + "/RobotController/CrawlGaitController.py")
CrawlStance_old = CrawlGait_old_mod.CrawlStanceController
CrawlSwing_old = CrawlGait_old_mod.CrawlSwingController

PID_old_mod = load_module_from_path("old_PID", OLD_DIR + "/RobotController/PIDController.py")
PID_old = PID_old_mod.PID_controller

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

def header(title):
    print("")
    print("=" * 78)
    print("  " + title)
    print("=" * 78)

def table_header():
    print("")
    print(" %-60s %-8s %s" % ("TEST", "STATUS", "DETAIL"))
    print("-" * 78)

def table_row(name, status, detail):
    icon = "[PASS]" if status == "PASS" else "[FAIL]" if status == "FAIL" else "[ERR]"
    print(" %-60s %-8s %s" % (name, icon, detail))

def table_footer():
    print("-" * 78)

class SimpleState:
    def __init__(self):
        self.foot_locations = np.array([
            [0.19, 0.19, -0.19, -0.19],
            [-0.15, 0.15, -0.15, 0.15],
            [-0.25, -0.25, -0.25, -0.25]
        ])
        self.robot_height = 0.25

class SimpleCommand:
    def __init__(self):
        self.velocity = np.array([0.0, 0.0])
        self.yaw_rate = np.array([0.0, 0.0, 0.0])
        self.robot_height = 0.25

# ============================================================================
# 1. TrotStanceController Tests (10 tests)
# ============================================================================
header("TROT STANCE CONTROLLER -- 10 tests")

ts_new = TS_new(phase_length=8, stance_ticks=4, swing_ticks=4, time_step=0.05, z_error_constant=0.02)
ts_old = TrotStance_old(phase_length=8, stance_ticks=4, swing_ticks=4, time_step=0.05, z_error_constant=0.02)

state = SimpleState()
cmd = SimpleCommand()

for leg in range(4):
    (dp_new, do_new) = ts_new.position_delta(leg, state, cmd)
    (dp_old, do_old) = ts_old.position_delta(leg, state, cmd)
    compare("TrotStance.position_delta(leg=%d)" % leg, dp_old, dp_new)

# Test with velocity
cmd.velocity = np.array([0.1, 0.0])
(dp_new, do_new) = ts_new.position_delta(0, state, cmd)
(dp_old, do_old) = ts_old.position_delta(0, state, cmd)
compare("TrotStance.position_delta(velocity)", dp_old, dp_new)

# Test next_foot_location
compare("TrotStance.next_foot_location(leg=0)", 
        ts_old.next_foot_location(0, state, cmd),
        ts_new.next_foot_location(0, state, cmd))

# ============================================================================
# 2. TrotSwingController Tests (8 tests)
# ============================================================================
header("TROT SWING CONTROLLER -- 8 tests")

default_stance = np.array([
    [0.19, 0.19, -0.19, -0.19],
    [-0.15, 0.15, -0.15, 0.15],
    [-0.25, -0.25, -0.25, -0.25]
])

tsw_new = TSw_new(stance_ticks=4, swing_ticks=4, time_step=0.05, phase_length=8, z_leg_lift=0.14, default_stance=default_stance)
tsw_old = TrotSwing_old(stance_ticks=4, swing_ticks=4, time_step=0.05, phase_length=8, z_leg_lift=0.14, default_stance=default_stance)

# Test swing_height
for phase in [0.0, 0.25, 0.5, 0.75, 1.0]:
    compare("TrotSwing.swing_height(%.2f)" % phase, tsw_old.swing_height(phase), tsw_new.swing_height(phase))

# Test raibert_touchdown_location
for leg in range(4):
    compare("TrotSwing.touchdown(leg=%d)" % leg, 
            tsw_old.raibert_touchdown_location(leg, cmd),
            tsw_new.raibert_touchdown_location(leg, cmd))

# Test next_foot_location
compare("TrotSwing.next_foot_location(phase=0.5)", 
        tsw_old.next_foot_location(0.5, 0, state, cmd),
        tsw_new.next_foot_location(0.5, 0, state, cmd))

# ============================================================================
# 3. CrawlStanceController Tests (8 tests)
# ============================================================================
header("CRAWL STANCE CONTROLLER -- 8 tests")

cs_new = CS_new(phase_length=12, stance_ticks=8, swing_ticks=4, time_step=0.05, z_error_constant=0.02, body_shift_y=0.03)
cs_old = CrawlStance_old(phase_length=12, stance_ticks=8, swing_ticks=4, time_step=0.05, z_error_constant=0.02, body_shift_y=0.03)

# Test position_delta with different parameters
for leg in range(4):
    (dp_new, do_new) = cs_new.position_delta(leg, state, cmd, first_cycle=False, move_sideways=False, move_left=True)
    (dp_old, do_old) = cs_old.position_delta(leg, state, cmd, first_cycle=False, move_sideways=False, move_left=True)
    compare("CrawlStance.position_delta(leg=%d)" % leg, dp_old, dp_new)

# Test with sideways movement
cmd.velocity = np.array([0.05, 0.0])
(dp_new, do_new) = cs_new.position_delta(0, state, cmd, first_cycle=True, move_sideways=True, move_left=True)
(dp_old, do_old) = cs_old.position_delta(0, state, cmd, first_cycle=True, move_sideways=True, move_left=True)
compare("CrawlStance.position_delta(sideways)", dp_old, dp_new)

# Test next_foot_location
compare("CrawlStance.next_foot_location", 
        cs_old.next_foot_location(0, state, cmd, first_cycle=False, move_sideways=False, move_left=True),
        cs_new.next_foot_location(0, state, cmd, first_cycle=False, move_sideways=False, move_left=True))

# ============================================================================
# 4. CrawlSwingController Tests (8 tests)
# ============================================================================
header("CRAWL SWING CONTROLLER -- 8 tests")

csw_new = CSw_new(stance_ticks=8, swing_ticks=4, time_step=0.05, phase_length=12, z_leg_lift=0.1, default_stance=default_stance, body_shift_y=0.03)
csw_old = CrawlSwing_old(stance_ticks=8, swing_ticks=4, time_step=0.05, phase_length=12, z_leg_lift=0.1, default_stance=default_stance, body_shift_y=0.03)

# Test swing_height
for phase in [0.0, 0.25, 0.5, 0.75, 1.0]:
    compare("CrawlSwing.swing_height(%.2f)" % phase, csw_old.swing_height(phase), csw_new.swing_height(phase))

# Test raibert_touchdown_location
cmd.velocity = np.array([0.05, 0.0])
for leg in range(4):
    compare("CrawlSwing.touchdown(leg=%d)" % leg, 
            csw_old.raibert_touchdown_location(leg, cmd, shifted_left=False),
            csw_new.raibert_touchdown_location(leg, cmd, shifted_left=False))

# Test next_foot_location
compare("CrawlSwing.next_foot_location(phase=0.5)", 
        csw_old.next_foot_location(0.5, 0, state, cmd, shifted_left=False),
        csw_new.next_foot_location(0.5, 0, state, cmd, shifted_left=False))

# ============================================================================
# 5. PIDController Tests (5 tests)
# ============================================================================
header("PID CONTROLLER -- 5 tests")

pid_new = PID_new(kp=0.15, ki=0.02, kd=0.002)
pid_old = PID_old(kp=0.15, ki=0.02, kd=0.002)

# Test compute_pid
for error in [0.0, 0.1, -0.1, 0.5]:
    compare("PID.compute(error=%.1f)" % error, pid_old.compute_pid(error), pid_new.compute_pid(error))

compare("PID.compute(large_error)", pid_old.compute_pid(1.0), pid_new.compute_pid(1.0))

# ============================================================================
# Print Summary Table
# ============================================================================
header("SUMMARY TABLE")

table_header()
for name, status, detail in results:
    table_row(name, status, detail)
table_footer()

# Group by module
passed_ts = sum(1 for n,s,d in results if s == "PASS" and "TrotStance" in n)
failed_ts = sum(1 for n,s,d in results if s == "FAIL" and "TrotStance" in n)
passed_tsw = sum(1 for n,s,d in results if s == "PASS" and "TrotSwing" in n)
failed_tsw = sum(1 for n,s,d in results if s == "FAIL" and "TrotSwing" in n)
passed_cs = sum(1 for n,s,d in results if s == "PASS" and "CrawlStance" in n)
failed_cs = sum(1 for n,s,d in results if s == "FAIL" and "CrawlStance" in n)
passed_csw = sum(1 for n,s,d in results if s == "PASS" and "CrawlSwing" in n)
failed_csw = sum(1 for n,s,d in results if s == "FAIL" and "CrawlSwing" in n)
passed_pid = sum(1 for n,s,d in results if s == "PASS" and "PID" in n)
failed_pid = sum(1 for n,s,d in results if s == "FAIL" and "PID" in n)

print("")
print("+================================================================================+")
print("|                              FINAL STATISTICS                                 |")
print("+================================================================================+")
print("|  TrotStanceController:     %2d passed  %2d failed                                   |" % (passed_ts, failed_ts))
print("|  TrotSwingController:     %2d passed  %2d failed                                   |" % (passed_tsw, failed_tsw))
print("|  CrawlStanceController:   %2d passed  %2d failed                                   |" % (passed_cs, failed_cs))
print("|  CrawlSwingController:    %2d passed  %2d failed                                   |" % (passed_csw, failed_csw))
print("|  PIDController:           %2d passed  %2d failed                                   |" % (passed_pid, failed_pid))
print("+================================================================================+")
print("|  TOTAL:                   %2d passed  %2d failed                                   |" % (passed, failed))
print("+================================================================================+")
print("")

sys.exit(0 if failed == 0 else 1)
"""

with open(test_script_path, "w") as f:
    f.write(test_code)
os.chmod(test_script_path, stat.S_IRWXU)

result = subprocess.run(
    [sys.executable, test_script_path], capture_output=True, text=True
)
print(result.stdout)
if result.returncode != 0:
    print("STDERR:", result.stderr[:2000])

shutil.rmtree(test_dir)

sys.exit(result.returncode)
