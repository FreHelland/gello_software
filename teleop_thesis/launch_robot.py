"""Main robot server for the real UR5e: same as launch_nodes.py --robot ur, plus the gripper's object detection status."""
from dataclasses import dataclass
from typing import Dict

import numpy as np
import tyro

from gello.robots.ur import URRobot
from gello.zmq_core.robot_node import ZMQServerRobot


@dataclass
class Args:
    robot_ip: str = "192.168.1.102"
    """The IP address of the UR5e."""

    robot_port: int = 6001
    """The port the robot server listens on."""

    hostname: str = "127.0.0.1"
    """The host the robot server binds to."""


class URRobotWithContact(URRobot):
    def get_observations(self) -> Dict[str, np.ndarray]:
        obs = super().get_observations()
        # 0 = moving, 1 = stopped on contact while opening, 2 = stopped on contact while closing, 3 = at requested position
        obs["gripper_object"] = np.array([self.gripper._get_var(self.gripper.OBJ)])
        return obs


def main(args: Args) -> None:
    robot = URRobotWithContact(robot_ip=args.robot_ip)
    server = ZMQServerRobot(robot, port=args.robot_port, host=args.hostname)
    print(f"Starting robot server on port {args.robot_port}")
    server.serve()


if __name__ == "__main__":
    main(tyro.cli(Args))
