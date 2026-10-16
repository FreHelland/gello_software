"""Teleop with the virtual wall on the leader trigger. Use with teleop_thesis/launch_ur_contact.py.

Same wall as gripper_wall.py: when the robot gripper stops on an object, the trigger motor holds the
trigger where the fingers stopped. Contact is taken from the gripper's flag or from the fingers stalling
while commanded further closed, whichever comes first. Opening the trigger past the fingers releases it.
"""
import csv
import os
import runpy
import sys
import time
from datetime import datetime

from gello.agents.gello_agent import GelloAgent
from gello.env import RobotEnv
from leader_config import GELLO_CONFIG, register
from leader_trigger import disable_spring, install_spring, read_present_current, set_current, set_goal_deg

WALL_CURRENT_MA = 1000
P_GAIN = 1600
D_GAIN = 1000
GRIP_MARGIN = 0.1
RELEASE_GAP = 0.05
STALL_TIME = 0.15
STALL_GAP = 0.03
CONTACT_STATES = (1, 2)
OUTDIR = "teleop_thesis/data"

_, OPEN_DEG, CLOSED_DEG = GELLO_CONFIG.gripper_config

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

_s = {"latched": False, "rearm": True, "wall": 0.0, "source": "", "last_pos": -1.0,
      "t_move": 0.0, "t_close": 0.0, "t0": None, "warned": False}
_rows = []
_original_act = GelloAgent.act


def to_deg(unit: float) -> float:
    return OPEN_DEG + unit * (CLOSED_DEG - OPEN_DEG)


def act_with_wall(self, obs):
    action = _original_act(self, obs)
    if "gripper_object" not in obs:
        if not _s["warned"]:
            print("\nno gripper_object in observations, start the server with launch_ur_contact.py")
            _s["warned"] = True
        return action

    now = time.time()
    leader = float(action[-1])
    follower = float(obs["gripper_position"][0])
    obj = int(obs["gripper_object"][0])

    if follower != _s["last_pos"]:
        if follower > _s["last_pos"] >= 0:
            _s["t_close"] = now
        _s["last_pos"], _s["t_move"] = follower, now
    # the gripper only raises its contact flag once the commands stop changing, so a stall is detected
    # here as well: fingers were just closing, have stopped, and are commanded further closed
    stalled = (
        now - _s["t_move"] >= STALL_TIME
        and now - _s["t_close"] < 1.0 + STALL_TIME
        and _s["t_close"] >= _s["t_move"]
        and leader - follower >= STALL_GAP
    )
    contact_now = obj in CONTACT_STATES or stalled

    if not contact_now:
        # the contact flag lags after a release, wait for it to clear before allowing a new contact
        _s["rearm"] = True
    if not _s["latched"] and _s["rearm"] and contact_now:
        _s.update(latched=True, wall=follower, source="flag" if obj in CONTACT_STATES else "stall")
        set_goal_deg(to_deg(follower))
        set_current(WALL_CURRENT_MA)
    elif _s["latched"] and (leader < _s["wall"] - RELEASE_GAP or follower > _s["wall"] + 0.03):
        # trigger opened past the wall, or the fingers moved on (object gone)
        _s.update(latched=False, rearm=False)
        set_current(0)

    if _s["latched"]:
        # keep the gripper closing on the object so it holds its grip and the flag stays on
        action = action.copy()
        action[-1] = max(leader, min(_s["wall"] + GRIP_MARGIN, 1.0))

    if _s["t0"] is None:
        _s["t0"] = now
    _rows.append(
        (f"{now - _s['t0']:.4f}", f"{leader:.4f}", f"{float(action[-1]):.4f}", f"{follower:.4f}", obj,
         int(_s["latched"]), f"{_s['wall']:.4f}" if _s["latched"] else "", _s["source"] if _s["latched"] else "",
         read_present_current())
    )
    return action


GelloAgent.act = act_with_wall


def save_log() -> None:
    if not _rows:
        return
    os.makedirs(OUTDIR, exist_ok=True)
    path = os.path.join(OUTDIR, f"teleop_wall_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    with open(path, "w", newline="") as f:
        f.write(
            f"# wall_mA={WALL_CURRENT_MA} p_gain={P_GAIN} d_gain={D_GAIN} grip_margin={GRIP_MARGIN} "
            f"release_gap={RELEASE_GAP} stall_time={STALL_TIME} stall_gap={STALL_GAP}\n"
        )
        writer = csv.writer(f)
        writer.writerow(["t_s", "leader_gripper", "gripper_cmd", "follower_gripper", "gripper_object",
                         "contact", "wall", "contact_source", "trigger_current_mA"])
        writer.writerows(_rows)
    print(f"\nlog saved to {path}")


install_spring(0, P_GAIN, D_GAIN)
register()
try:
    runpy.run_path("experiments/run_env.py", run_name="__main__")
except KeyboardInterrupt:
    pass
finally:
    disable_spring()
    save_log()
