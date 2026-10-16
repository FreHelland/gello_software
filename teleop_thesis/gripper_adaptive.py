"""Gripper only, adaptive grip force with a moving wall on the leader trigger. The arm is never commanded.

After contact, the gripper's force setting follows the gap between the leader trigger and the gripper
fingers: squeeze further past the fingers and the gripper pushes harder. A rigid object stops the
fingers whatever the force, a soft one lets them close further. The trigger wall follows the fingers,
so the trigger gives way with a soft object and stays put with a rigid one.

Pendant: arm powered on, gripper activated, emergency release off. No other program connected to the gripper.
"""
import csv
import os
import time
from dataclasses import dataclass
from datetime import datetime

import numpy as np
import tyro

from gello.dynamixel.driver import DynamixelDriver
from gello.robots.robotiq_gripper import RobotiqGripper
from leader_config import GELLO_CONFIG, PORT
from leader_trigger import (
    disable_spring,
    enable_spring,
    get_p_gain,
    read_present_current,
    set_current,
    set_goal_deg,
    set_p_gain,
)

CONTACT_STATES = (1, 2)
AT_REQUESTED_POSITION = 3


@dataclass
class Args:
    robot_ip: str = "192.168.1.102"
    """The IP address of the UR5e (the gripper is reached through it)."""

    gripper_port: int = 63352
    """The Robotiq socket port on the controller."""

    speed: int = 255
    """Gripper speed (0-255)."""

    free_force: int = 10
    """Gripper force setting (0-255) before contact."""

    min_force: int = 10
    """Gripper force setting at contact, gap at or below min_gap."""

    max_force: int = 150
    """Gripper force setting at full_force_gap and above (255 is the gripper's maximum)."""

    min_gap: float = 0.02
    """Gap below which the force stays at min_force (0-1 scale)."""

    full_force_gap: float = 0.25
    """Gap at which the force reaches max_force (0-1 scale)."""

    force_step: int = 5
    """Only resend the gripper command when the force setting changes by at least this."""

    wall_current: int = 1000
    """Trigger motor current cap at the wall in mA (max 1000)."""

    p_gain: int = 1000
    """Trigger motor position P gain while running (wall stiffness), 0 keeps the motor's own value."""

    wall_step: float = 0.005
    """Move the wall when the fingers have moved by more than this (0-1 scale)."""

    release_gap: float = 0.05
    """Open the trigger this far past the fingers to release (0-1 scale)."""

    command_deadband: float = 0.01
    """Only resend the gripper command when the position changes by more than this (0-1 scale)."""

    hz: float = 50.0
    """Loop rate upper limit."""

    label: str = ""
    """Optional tag added to the output file name, e.g. the object."""

    outdir: str = "teleop_thesis/data"
    """Where the log is saved."""


def grip_force(gap: float, args: Args) -> int:
    if gap <= args.min_gap:
        return args.min_force
    fraction = min((gap - args.min_gap) / (args.full_force_gap - args.min_gap), 1.0)
    return int(round(args.min_force + (args.max_force - args.min_force) * fraction))


def run(args: Args) -> None:
    assert 0 < args.wall_current <= 1000
    assert 0 <= args.min_force <= args.max_force <= 255
    assert 0 <= args.min_gap < args.full_force_gap
    gripper_id, open_deg, closed_deg = GELLO_CONFIG.gripper_config

    def to_unit(deg: float) -> float:
        return float(np.clip((deg - open_deg) / (closed_deg - open_deg), 0.0, 1.0))

    def to_deg(unit: float) -> float:
        return open_deg + unit * (closed_deg - open_deg)

    driver = DynamixelDriver([gripper_id], port=PORT, baudrate=57600, use_fake_fallback=False)
    # torque on with zero current, so the trigger is free until contact
    enable_spring(driver, 0)
    original_p_gain = get_p_gain()
    if args.p_gain > 0:
        set_p_gain(args.p_gain)
    print(f"trigger position P gain: {get_p_gain()} (motor's own value {original_p_gain})")

    gripper = RobotiqGripper()
    gripper.connect(hostname=args.robot_ip, port=args.gripper_port)
    print("gripper connected, status:", gripper._get_var(gripper.STA), "(3 = active)")

    rows = []
    t0 = time.time()
    period = 1.0 / args.hz
    latched = False
    rearm = True
    wall = 0.0
    follower = 0.0
    last_cmd, last_force = -1.0, -1
    print("running, Ctrl+C to stop")
    try:
        while True:
            t_start = time.time()
            trigger_deg = float(np.rad2deg(driver.get_joints()[0]))
            leader = to_unit(trigger_deg)

            gap = leader - follower
            force = grip_force(gap, args) if latched else args.free_force
            if (
                abs(leader - last_cmd) > args.command_deadband
                or leader in (0.0, 1.0) and leader != last_cmd
                or abs(force - last_force) >= args.force_step
            ):
                gripper.move(int(round(leader * 255)), args.speed, force)
                last_cmd, last_force = leader, force
            pos = gripper._get_var(gripper.POS)
            obj = gripper._get_var(gripper.OBJ)
            # unit is 10 mA
            gripper_ma = gripper._get_var("COU") * 10
            follower = pos / 255

            if obj not in CONTACT_STATES:
                # the contact flag lags after a release, wait for it to clear before allowing a new contact
                rearm = True
            if not latched and rearm and obj in CONTACT_STATES:
                latched, wall = True, follower
                set_goal_deg(to_deg(wall))
                set_current(args.wall_current)
            elif latched and (leader < follower - args.release_gap or obj == AT_REQUESTED_POSITION):
                # trigger opened past the fingers, or the fingers reached the trigger (nothing resists any more)
                latched, rearm = False, False
                set_current(0)
            elif latched and abs(follower - wall) > args.wall_step:
                # a soft object gives way, the wall follows the fingers
                wall = follower
                set_goal_deg(to_deg(wall))
            trigger_ma = read_present_current()

            rows.append(
                (f"{t_start - t0:.4f}", f"{trigger_deg:.2f}", f"{leader:.4f}", f"{follower:.4f}",
                 f"{leader - follower:.4f}", obj, int(latched), f"{wall:.4f}" if latched else "",
                 last_force, gripper_ma, args.wall_current if latched else 0, trigger_ma)
            )
            print(f"\rleader {leader:5.3f}  gripper {follower:5.3f}  obj {obj}  wall {'ON ' if latched else 'off'}"
                  f"  force {last_force:3d}  trigger {trigger_ma:5d} mA   ", end="")
            time.sleep(max(0.0, period - (time.time() - t_start)))
    except KeyboardInterrupt:
        print()
    finally:
        try:
            set_p_gain(original_p_gain)
            disable_spring()
        finally:
            driver.close()
            gripper.disconnect()

    if not rows:
        return
    os.makedirs(args.outdir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    label = f"_{args.label}" if args.label else ""
    path = os.path.join(args.outdir, f"gripper_adaptive{label}_{stamp}.csv")
    with open(path, "w", newline="") as f:
        f.write(
            f"# speed={args.speed} free_force={args.free_force} min_force={args.min_force} "
            f"max_force={args.max_force} min_gap={args.min_gap} full_force_gap={args.full_force_gap} "
            f"wall_mA={args.wall_current} p_gain={args.p_gain or original_p_gain} release_gap={args.release_gap} "
            f"hz={args.hz} open_deg={open_deg} closed_deg={closed_deg}\n"
        )
        writer = csv.writer(f)
        writer.writerow(["t_s", "trigger_deg", "leader_gripper", "follower_gripper", "gap",
                         "gripper_object", "contact", "wall", "gripper_force_cmd",
                         "gripper_current_mA", "trigger_cap_mA", "trigger_current_mA"])
        writer.writerows(rows)
    print("saved to: ", path)


def main(args: Args) -> None:
    run(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
