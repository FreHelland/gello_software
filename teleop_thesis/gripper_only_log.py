"""Leader trigger drives only the Robotiq gripper, the arm is never commanded. Logs positions, gap and gripper current.

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


@dataclass
class Args:
    robot_ip: str = "192.168.1.102"
    """The IP address of the UR5e (the gripper is reached through it)."""

    gripper_port: int = 63352
    """The Robotiq socket port on the controller."""

    speed: int = 255
    """Gripper speed (0-255)."""

    force: int = 10
    """Gripper grip force (0-255), 10 is what upstream teleop uses."""

    hz: float = 50.0
    """Loop rate upper limit."""

    label: str = ""
    """Optional tag added to the output file name, e.g. the object."""

    outdir: str = "teleop_thesis/data"
    """Where the log is saved."""


def to_unit(deg: float, open_deg: float, closed_deg: float) -> float:
    return float(np.clip((deg - open_deg) / (closed_deg - open_deg), 0.0, 1.0))


def run(args: Args) -> None:
    gripper_id, open_deg, closed_deg = GELLO_CONFIG.gripper_config
    # read only, the driver turns torque off on the trigger motor at start
    driver = DynamixelDriver([gripper_id], port=PORT, baudrate=57600, use_fake_fallback=False)

    gripper = RobotiqGripper()
    gripper.connect(hostname=args.robot_ip, port=args.gripper_port)
    print("gripper connected, status:", gripper._get_var(gripper.STA), "(3 = active)")

    has_current = True
    try:
        gripper._get_var("COU")
    except Exception as e:
        has_current = False
        print("gripper current (COU) not readable, logging without it:", e)

    rows = []
    t0 = time.time()
    period = 1.0 / args.hz
    print("running, Ctrl+C to stop")
    try:
        while True:
            t_start = time.time()
            trigger_deg = float(np.rad2deg(driver.get_joints()[0]))
            leader = to_unit(trigger_deg, open_deg, closed_deg)
            gripper.move(int(round(leader * 255)), args.speed, args.force)
            pos = gripper._get_var(gripper.POS)
            obj = gripper._get_var(gripper.OBJ)
            # unit is 10 mA
            current_ma = gripper._get_var("COU") * 10 if has_current else -1
            follower = pos / 255
            gap = leader - follower
            rows.append(
                (f"{t_start - t0:.4f}", f"{trigger_deg:.2f}", f"{leader:.4f}", pos,
                 f"{follower:.4f}", f"{gap:.4f}", obj, current_ma)
            )
            print(f"\rleader {leader:5.3f}  gripper {follower:5.3f}  gap {gap:+6.3f}  obj {obj}  current {current_ma:4d} mA   ", end="")
            time.sleep(max(0.0, period - (time.time() - t_start)))
    except KeyboardInterrupt:
        print()
    finally:
        driver.close()
        gripper.disconnect()

    if not rows:
        return
    os.makedirs(args.outdir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    label = f"_{args.label}" if args.label else ""
    path = os.path.join(args.outdir, f"gripper_only{label}_{stamp}.csv")
    with open(path, "w", newline="") as f:
        f.write(
            f"# speed={args.speed} force={args.force} hz={args.hz} open_deg={open_deg} "
            f"closed_deg={closed_deg} current_readable={has_current}\n"
        )
        writer = csv.writer(f)
        writer.writerow(["t_s", "trigger_deg", "leader_gripper", "gripper_pos_raw",
                         "follower_gripper", "gap", "gripper_object", "gripper_current_mA"])
        writer.writerows(rows)
    print("saved to: ", path)


def main(args: Args) -> None:
    run(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
