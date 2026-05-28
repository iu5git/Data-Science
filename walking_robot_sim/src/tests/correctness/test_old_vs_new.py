#!/usr/bin/env python3
"""
Comprehensive тест old vs new — используем importlib.
Красивые таблицы без Unicode проблем.
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

final_path = "/tmp/test_modules_correct"
if os.path.exists(final_path):
    shutil.rmtree(final_path)
shutil.copytree(modules_dir, final_path)

test_script_path = os.path.join(test_dir, "run_test.py")

test_code = r"""#!/usr/bin/env python3
import sys
import os
import math
import numpy as np
import importlib.util
import re

def load_module_from_path(module_name, filepath):
    spec = importlib.util.spec_from_file_location(module_name, filepath)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module

sys.path.insert(0, "/tmp/test_modules_correct/RoboticsUtilities")
sys.path.insert(0, "/tmp/test_modules_correct")

from ForwardKinematics.forward_kinematics import ForwardKinematics as FK_new
from ForwardKinematics.leg_base_positions import get_leg_base_position as get_leg_base_new
from ForwardKinematics.leg_fk_chain import compute_leg_fk_chain as leg_fk_new
from InverseKinematics.inverse_kinematics import InverseKinematics as IK_new
from InverseKinematics.local_positions import compute_local_positions as local_pos_new
from InverseKinematics.joint_angles import compute_all_joint_angles as joint_angles_new
from RoboticsUtilities.rotation_matrices import rotxyz, rotx, roty, rotz
from RoboticsUtilities.homogeneous_transforms import homog_transform, homog_transform_inverse

OLD_DIR = "/home/redalexdad/GitHub/WalkingRobotSim/src/tests/old"

FK_old_mod = load_module_from_path("old_FK", OLD_DIR + "/ForwardKinematics/robot_FK.py")
FK_old = FK_old_mod.ForwardKinematics

IK_old_mod = load_module_from_path("old_IK", OLD_DIR + "/InverseKinematics/robot_IK.py")
IK_old = IK_old_mod.InverseKinematics

Trans_old_mod = load_module_from_path("old_Transformations", OLD_DIR + "/RoboticsUtilities/Transformations.py")
rotxyz_old = Trans_old_mod.rotxyz
rotx_old = Trans_old_mod.rotx
roty_old = Trans_old_mod.roty
rotz_old = Trans_old_mod.rotz
homog_transform_old = Trans_old_mod.homog_transform
homog_transform_inverse_old = Trans_old_mod.homog_transform_inverse

body_dim = [0.3762, 0.0935]
leg_dim = [0.0, 0.0955, 0.213, 0.213]

FK_old_inst = FK_old(body_dim, leg_dim)
IK_old_inst = IK_old(body_dim, leg_dim)

FK_new = FK_new(body_dim, leg_dim)
IK_new = IK_new(body_dim, leg_dim)

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

# ============================================================================
# ForwardKinematics Tests
# ============================================================================
header("FORWARDKINEMATICS -- 15 tests")

angles = [0, 0.3, -0.6] * 4
compare("FK.forward_kinematics_all_legs(zero)", FK_old_inst.forward_kinematics_all_legs(angles), FK_new.forward_kinematics_all_legs(angles))

angles2 = [0.1, -0.2, 0.5] * 4
compare("FK.forward_kinematics_all_legs(random)", FK_old_inst.forward_kinematics_all_legs(angles2), FK_new.forward_kinematics_all_legs(angles2))

for leg in range(4):
    compare("FK.forward_kinematics_per_leg(%d)" % leg, 
            FK_old_inst.forward_kinematics_per_leg(0.2, -0.3, 0.5, leg),
            FK_new.forward_kinematics_per_leg(0.2, -0.3, 0.5, leg))

compare("FK.forward_kinematics_per_leg(angles=0.5,-0.8,0.3)", 
        FK_old_inst.forward_kinematics_per_leg(0.5, -0.8, 0.3, 0),
        FK_new.forward_kinematics_per_leg(0.5, -0.8, 0.3, 0))

compare("FK.forward_kinematics_per_leg(angles=-0.4,0.6,-0.2)", 
        FK_old_inst.forward_kinematics_per_leg(-0.4, 0.6, -0.2, 1),
        FK_new.forward_kinematics_per_leg(-0.4, 0.6, -0.2, 1))

compare("FK.homog_transform(0,0,0,0,0,0)", 
        FK_old_inst.homog_transform(0, 0, 0, 0, 0, 0),
        FK_new.homog_transform(0, 0, 0, 0, 0, 0))

compare("FK.homog_transform(0.1,0.2,0.3,0.4,0.5,0.6)", 
        FK_old_inst.homog_transform(0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
        FK_new.homog_transform(0.1, 0.2, 0.3, 0.4, 0.5, 0.6))

compare("FK.homog_transform(negative)", 
        FK_old_inst.homog_transform(-0.1, -0.2, -0.3, -0.4, -0.5, -0.6),
        FK_new.homog_transform(-0.1, -0.2, -0.3, -0.4, -0.5, -0.6))

# ============================================================================
# InverseKinematics Tests
# ============================================================================
header("INVERSEKINEMATICS -- 10 tests")

dx = 0.3762 * 0.5 + 0.02
dy = 0.0935 * 0.5 + 0.0955

stance1 = np.array([[dx, dx, -dx, -dx], [-dy, dy, -dy, dy], [0, 0, 0, 0]])
compare("IK.inverse_kinematics(roll=0)", 
        IK_old_inst.inverse_kinematics(stance1, 0, 0, 0.25, 0, 0, 0),
        IK_new.inverse_kinematics(stance1, 0, 0, 0.25, 0, 0, 0))

compare("IK.inverse_kinematics(roll=0.1)", 
        IK_old_inst.inverse_kinematics(stance1, 0, 0, 0.25, 0.1, 0, 0),
        IK_new.inverse_kinematics(stance1, 0, 0, 0.25, 0.1, 0, 0))

compare("IK.inverse_kinematics(pitch=0.1)", 
        IK_old_inst.inverse_kinematics(stance1, 0, 0, 0.25, 0, 0.1, 0),
        IK_new.inverse_kinematics(stance1, 0, 0, 0.25, 0, 0.1, 0))

compare("IK.inverse_kinematics(yaw=0.1)", 
        IK_old_inst.inverse_kinematics(stance1, 0, 0, 0.25, 0, 0, 0.1),
        IK_new.inverse_kinematics(stance1, 0, 0, 0.25, 0, 0, 0.1))

stance2 = np.array([[dx, dx, -dx, -dx], [-dy, dy, -dy, dy], [-0.1, -0.1, -0.1, -0.1]])
compare("IK.inverse_kinematics(height=-0.1)", 
        IK_old_inst.inverse_kinematics(stance2, 0, 0, 0.25, 0, 0, 0),
        IK_new.inverse_kinematics(stance2, 0, 0, 0.25, 0, 0, 0))

stance3 = np.array([[dx, dx, -dx, -dx], [-dy, dy, -dy, dy], [0.1, 0.1, 0.1, 0.1]])
compare("IK.inverse_kinematics(height=0.1)", 
        IK_old_inst.inverse_kinematics(stance3, 0, 0, 0.25, 0, 0, 0),
        IK_new.inverse_kinematics(stance3, 0, 0, 0.25, 0, 0, 0))

compare("IK.inverse_kinematics(roll=0.05,pitch=0.03,yaw=0.02)", 
        IK_old_inst.inverse_kinematics(stance1, 0, 0, 0.25, 0.05, 0.03, 0.02),
        IK_new.inverse_kinematics(stance1, 0, 0, 0.25, 0.05, 0.03, 0.02))

compare("IK.inverse_kinematics(negative orientation)", 
        IK_old_inst.inverse_kinematics(stance1, 0, 0, 0.25, -0.05, -0.03, -0.02),
        IK_new.inverse_kinematics(stance1, 0, 0, 0.25, -0.05, -0.03, -0.02))

compare("IK.inverse_kinematics(roll=0.2)", 
        IK_old_inst.inverse_kinematics(stance1, 0, 0, 0.25, 0.2, 0, 0),
        IK_new.inverse_kinematics(stance1, 0, 0, 0.25, 0.2, 0, 0))

# ============================================================================
# RoboticsUtilities Tests
# ============================================================================
header("ROBOTICSUTILITIES -- 12 tests")

compare("rotxyz(0,0,0)", rotxyz_old(0, 0, 0), rotxyz(0, 0, 0))
compare("rotxyz(0.1,-0.05,0.02)", rotxyz_old(0.1, -0.05, 0.02), rotxyz(0.1, -0.05, 0.02))
compare("rotxyz(pi/4,pi/6,pi/3)", rotxyz_old(math.pi/4, math.pi/6, math.pi/3), rotxyz(math.pi/4, math.pi/6, math.pi/3))
compare("rotxyz(negative)", rotxyz_old(-0.1, -0.2, -0.3), rotxyz(-0.1, -0.2, -0.3))
compare("rotxyz(large)", rotxyz_old(1.0, 0.5, 0.8), rotxyz(1.0, 0.5, 0.8))

compare("roty(0.3)", roty_old(0.3), roty(0.3))
compare("roty(-0.5)", roty_old(-0.5), roty(-0.5))

compare("rotz(0.2)", rotz_old(0.2), rotz(0.2))
compare("rotz(-0.4)", rotz_old(-0.4), rotz(-0.4))

compare("homog_transform", homog_transform_old(0.1, 0.2, 0.3, 0.4, 0.5, 0.6), homog_transform(0.1, 0.2, 0.3, 0.4, 0.5, 0.6))

m = homog_transform(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
compare("homog_transform_inverse", homog_transform_inverse_old(m.copy()), homog_transform_inverse(m.copy()))

m2 = homog_transform(0, 0, 0, 0.2, 0.1, 0)
compare("homog_transform_inverse(identity-like)", homog_transform_inverse_old(m2.copy()), homog_transform_inverse(m2.copy()))

# ============================================================================
# Print Summary Table
# ============================================================================
header("SUMMARY TABLE")

table_header()
for name, status, detail in results:
    table_row(name, status, detail)
table_footer()

passed_fk = sum(1 for n,s,d in results if s == "PASS" and "FK." in n)
failed_fk = sum(1 for n,s,d in results if s == "FAIL" and "FK." in n)
passed_ik = sum(1 for n,s,d in results if s == "PASS" and "IK." in n)
failed_ik = sum(1 for n,s,d in results if s == "FAIL" and "IK." in n)
passed_ru = sum(1 for n,s,d in results if s == "PASS" and ("rot" in n or "homog" in n))
failed_ru = sum(1 for n,s,d in results if s == "FAIL" and ("rot" in n or "homog" in n))

print("")
print("+================================================================================+")
print("|                              FINAL STATISTICS                                 |")
print("+================================================================================+")
print("|  ForwardKinematics:    %2d passed  %2d failed                                   |" % (passed_fk, failed_fk))
print("|  InverseKinematics:    %2d passed  %2d failed                                   |" % (passed_ik, failed_ik))
print("|  RoboticsUtilities:    %2d passed  %2d failed                                   |" % (passed_ru, failed_ru))
print("+================================================================================+")
print("|  TOTAL:                %2d passed  %2d failed                                   |" % (passed, failed))
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
    print("STDERR:", result.stderr)

shutil.rmtree(test_dir)

sys.exit(result.returncode)
