"""Teleop with grip feedback on the leader trigger.

Without contact the trigger motor gives no force (the physical spring alone returns the trigger).
When the robot gripper reports contact with an object, the trigger current rises from 0 to MAX_CURRENT_MA
with the position gap between the leader trigger and the robot gripper. Use with teleop_thesis/launch_ur_contact.py.

Contact is latched: the gripper's flag drops to "moving" whenever a new position command arrives, even with
the fingers stopped on the object, so contact is kept until the gap closes or the gripper reports no object.

Own flags (removed before upstream run_env.py sees the arguments):
    --log-only    log the signals, keep the spring at the base current
"""
import csv
import os
import runpy
import sys
import time
from datetime import datetime

import numpy as np

from gello.agents.gello_agent import GelloAgent
from gello.env import RobotEnv
from leader_config import register
from leader_trigger import disable_spring, get_current, install_spring, set_current

BASE_CURRENT_MA = 0
MAX_CURRENT_MA = 150
MIN_GAP = 0.02
FULL_FORCE_GAP = 0.25
MIN_CURRENT_STEP_MA = 5
CONTACT_STATES = (1, 2)
NO_OBJECT_STATE = 3
OUTDIR = "teleop_thesis/data"

LOG_ONLY = "--log-only" in sys.argv
sys.argv = [a for a in sys.argv if a != "--log-only"]

assert 0 <= BASE_CURRENT_MA <= MAX_CURRENT_MA <= 300
assert 0 <= MIN_GAP < FULL_FORCE_GAP

# default to the real leader arm, upstream's default agent is a dummy that reads all zeros
if not any(a == "--agent" or a.startswith("--agent=") for a in sys.argv[1:]):
    sys.argv.append("--agent=gello")


def get_obs_all_keys(self):
    # upstream get_obs keeps only a fixed set of keys, this passes everything the robot server sends
    observations = {}
    for name, camera in self._camera_dict.items():
        image, depth = camera.read()
        observations[f"{name}_rgb"] = image
        observations[f"{name}_depth"] = depth
    observations.update(self._robot.get_observations())
    assert "joint_positions" in observations
    return observations


RobotEnv.get_obs = get_obs_all_keys

_log = {"rows": [], "t0": None, "warned": False}
_original_act = GelloAgent.act


_contact = {"latched": False}


def update_contact(gap: float, obj: int) -> bool:
    if obj in CONTACT_STATES:
        _contact["latched"] = True
    elif obj == NO_OBJECT_STATE or gap <= MIN_GAP:
        _contact["latched"] = False
    return _contact["latched"]


def target_current(gap: float, contact: bool) -> int:
    if not contact or gap <= MIN_GAP:
        return BASE_CURRENT_MA
    # linear from BASE at MIN_GAP to MAX at FULL_FORCE_GAP, flat above
    fraction = min((gap - MIN_GAP) / (FULL_FORCE_GAP - MIN_GAP), 1.0)
    return int(round(BASE_CURRENT_MA + (MAX_CURRENT_MA - BASE_CURRENT_MA) * fraction))


def act_with_feedback(self, obs):
    action = _original_act(self, obs)
    if "gripper_object" not in obs:
        if not _log["warned"]:
            print("\nno gripper_object in observations, start the server with launch_ur_contact.py")
            _log["warned"] = True
        return action

    leader_g = float(action[-1])
    follower_g = float(obs["gripper_position"][0])
    obj = int(obs["gripper_object"][0])
    gap = leader_g - follower_g
    contact = update_contact(gap, obj)
    target = BASE_CURRENT_MA if LOG_ONLY else target_current(gap, contact)
    if get_current() is not None and abs(target - get_current()) >= MIN_CURRENT_STEP_MA:
        set_current(target)

    now = time.time()
    if _log["t0"] is None:
        _log["t0"] = now
    _log["rows"].append(
        (f"{now - _log['t0']:.4f}", f"{leader_g:.4f}", f"{follower_g:.4f}", obj, int(contact),
         f"{gap:.4f}", target, get_current())
    )
    return action


GelloAgent.act = act_with_feedback


def save_log() -> None:
    if not _log["rows"]:
        return
    os.makedirs(OUTDIR, exist_ok=True)
    mode = "logonly" if LOG_ONLY else "feedback"
    path = os.path.join(OUTDIR, f"{mode}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    with open(path, "w", newline="") as f:
        f.write(
            f"# base_mA={BASE_CURRENT_MA} max_mA={MAX_CURRENT_MA} min_gap={MIN_GAP} "
            f"full_force_gap={FULL_FORCE_GAP} log_only={LOG_ONLY}\n"
        )
        writer = csv.writer(f)
        writer.writerow(["t_s", "leader_gripper", "follower_gripper", "gripper_object",
                         "contact", "gap", "target_mA", "commanded_mA"])
        writer.writerows(_log["rows"])
    print(f"\nlog saved to {path}")


install_spring(BASE_CURRENT_MA)
register()
try:
    runpy.run_path("experiments/run_env.py", run_name="__main__")
except KeyboardInterrupt:
    pass
finally:
    disable_spring()
    save_log()
