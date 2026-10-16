"""Gripper only, with a virtual wall on the leader trigger. The arm is never commanded.

When the Robotiq reports contact, the trigger motor holds the trigger where the fingers stopped, so the
object is felt as a wall. Opening the trigger past the wall releases it. Logs positions, contact,
gripper current and the trigger motor's current (how hard the hand pushes into the wall).

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
    get_d_gain,
    get_p_gain,
    read_present_current,
    set_current,
    set_d_gain,
    set_goal_deg,
    set_p_gain,
)

CONTACT_STATES = (1, 2)


@dataclass
class Args:
    robot_ip: str = "192.168.1.102"
    """The IP address of the UR5e (the gripper is reached through it)."""

    gripper_port: int = 63352
    """The Robotiq socket port on the controller."""

    speed: int = 255
    """Gripper speed (0-255)."""

    force: int = 10
    """Gripper grip force (0-255)."""

    wall_current: int = 1000
    """Trigger motor current cap at the wall in mA, how hard the wall pushes back (max 1000)."""

    grip_margin: float = 0.1
    """While in contact the gripper is commanded this far past the wall so it keeps its grip."""

    release_gap: float = 0.05
    """Open the trigger this far past the wall to release (0-1 scale)."""

    p_gain: int = 1000
    """Trigger motor position P gain while running (wall stiffness), 0 keeps the motor's own value."""

    d_gain: int = 1000
    """Trigger motor position D gain while running (wall damping), 0 keeps the motor's own value."""

    stall_time: float = 0.15
    """Fingers unchanged this long (s) while commanded further closed counts as contact."""

    stall_gap: float = 0.03
    """How far past the fingers the command must be for a stall to count as contact (0-1 scale)."""

    command_deadband: float = 0.01
    """Only send a new gripper command when it changes by more than this (0-1 scale)."""

    hz: float = 50.0
    """Loop rate upper limit."""

    label: str = ""
    """Optional tag added to the output file name, e.g. the object."""

    outdir: str = "teleop_thesis/data"
    """Where the log is saved."""


def stall_time_ok(still_for: float, stall_time: float) -> bool:
    return still_for >= stall_time


def run(args: Args) -> None:
    assert 0 < args.wall_current <= 1000
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
    original_d_gain = get_d_gain()
    if args.d_gain > 0:
        set_d_gain(args.d_gain)
    print(f"trigger position P gain: {get_p_gain()} (motor's own value {original_p_gain})")
    print(f"trigger position D gain: {get_d_gain()} (motor's own value {original_d_gain})")

    gripper = RobotiqGripper()
    gripper.connect(hostname=args.robot_ip, port=args.gripper_port)
    print("gripper connected, status:", gripper._get_var(gripper.STA), "(3 = active)")

    rows = []
    t0 = time.time()
    period = 1.0 / args.hz
    latched = False
    rearm = True
    wall = 0.0
    last_pos, t_last_move, t_last_close = -1, 0.0, 0.0
    source = ""
    last_cmd = -1.0
    print("running, Ctrl+C to stop")
    try:
        while True:
            t_start = time.time()
            trigger_deg = float(np.rad2deg(driver.get_joints()[0]))
            leader = to_unit(trigger_deg)

            cmd = max(leader, min(wall + args.grip_margin, 1.0)) if latched else leader
            if abs(cmd - last_cmd) > args.command_deadband or cmd in (0.0, 1.0) and cmd != last_cmd:
                gripper.move(int(round(cmd * 255)), args.speed, args.force)
                last_cmd = cmd
            pos = gripper._get_var(gripper.POS)
            obj = gripper._get_var(gripper.OBJ)
            # unit is 10 mA
            gripper_ma = gripper._get_var("COU") * 10
            follower = pos / 255

            now = time.time()
            if pos != last_pos:
                if pos > last_pos >= 0:
                    t_last_close = now
                last_pos, t_last_move = pos, now
            # the gripper only raises its contact flag once the commands stop changing, so a stall is
            # detected here as well: fingers were closing, have stopped, and are commanded further closed
            stalled = (
                stall_time_ok(now - t_last_move, args.stall_time)
                # only right after the fingers were closing, not after opening or a long rest
                and now - t_last_close < 1.0 + args.stall_time
                and t_last_close >= t_last_move
                and cmd - follower >= args.stall_gap
            )
            contact_now = obj in CONTACT_STATES or stalled

            if not contact_now:
                # the contact flag lags after a release, wait for it to clear before allowing a new contact
                rearm = True
            if not latched and rearm and contact_now:
                latched, wall = True, follower
                source = "flag" if obj in CONTACT_STATES else "stall"
                set_goal_deg(to_deg(wall))
                set_current(args.wall_current)
            elif latched and (leader < wall - args.release_gap or follower > wall + 0.03):
                # trigger opened past the wall, or the fingers moved on (object gone)
                latched, rearm = False, False
                set_current(0)
            trigger_ma = read_present_current()

            rows.append(
                (f"{t_start - t0:.4f}", f"{trigger_deg:.2f}", f"{leader:.4f}", f"{cmd:.4f}",
                 f"{follower:.4f}", obj, gripper_ma, int(latched), f"{wall:.4f}" if latched else "",
                 args.wall_current if latched else 0, trigger_ma, source if latched else "")
            )
            print(f"\rleader {leader:5.3f}  gripper {follower:5.3f}  obj {obj}  wall {'ON ' if latched else 'off'}"
                  f"  trigger {trigger_ma:5d} mA   ", end="")
            time.sleep(max(0.0, period - (time.time() - t_start)))
    except KeyboardInterrupt:
        print()
    finally:
        try:
            set_p_gain(original_p_gain)
            set_d_gain(original_d_gain)
            disable_spring()
        finally:
            driver.close()
            gripper.disconnect()

    if not rows:
        return
    os.makedirs(args.outdir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    label = f"_{args.label}" if args.label else ""
    path = os.path.join(args.outdir, f"gripper_wall{label}_{stamp}.csv")
    with open(path, "w", newline="") as f:
        f.write(
            f"# speed={args.speed} force={args.force} wall_mA={args.wall_current} p_gain={args.p_gain or original_p_gain} "
            f"d_gain={args.d_gain or original_d_gain} stall_time={args.stall_time} stall_gap={args.stall_gap} "
            f"grip_margin={args.grip_margin} "
            f"release_gap={args.release_gap} deadband={args.command_deadband} hz={args.hz} "
            f"open_deg={open_deg} closed_deg={closed_deg}\n"
        )
        writer = csv.writer(f)
        writer.writerow(["t_s", "trigger_deg", "leader_gripper", "gripper_cmd", "follower_gripper",
                         "gripper_object", "gripper_current_mA", "contact", "wall",
                         "trigger_cap_mA", "trigger_current_mA", "contact_source"])
        writer.writerows(rows)
    print("saved to: ", path)


def main(args: Args) -> None:
    run(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
