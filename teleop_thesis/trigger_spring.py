import time
from dataclasses import dataclass

import numpy as np
import tyro
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.robotis_def import COMM_SUCCESS

# Constants (XL330 control table)
ADDR_OPERATING_MODE = 11
ADDR_CURRENT_LIMIT = 38
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132
CURRENT_BASED_POSITION_MODE = 5
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


@dataclass
class Args:
    port: str = (
        "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAK8B0I-if00-port0"
    )
    """The port that GELLO is connected to."""

    baud: int = 57600
    """The baudrate of the dynamixel bus."""

    gripper_id: int = 7
    """The dynamixel id of the trigger."""

    open_deg: float = 190.0
    """Trigger position the spring pulls towards (in degrees, raw motor angle)."""

    goal_current: int = 20
    """Current cap in mA, sets how hard the spring can push."""

    max_current: int = 150
    """Refuse to run above this current (in mA)."""

    duration: float = 30.0
    """How long the spring is on (in seconds)."""


class Motor:
    def __init__(self, port_handler, packet_handler, dxl_id: int):
        self._port = port_handler
        self._packet = packet_handler
        self._id = dxl_id

    def _check(self, dxl_comm_result, dxl_error, what: str) -> None:
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            raise RuntimeError(f"Failed to {what} for Dynamixel with ID {self._id}")

    def read1(self, addr: int) -> int:
        value, res, err = self._packet.read1ByteTxRx(self._port, self._id, addr)
        self._check(res, err, f"read address {addr}")
        return value

    def read2(self, addr: int) -> int:
        value, res, err = self._packet.read2ByteTxRx(self._port, self._id, addr)
        self._check(res, err, f"read address {addr}")
        return value

    def read4(self, addr: int) -> int:
        value, res, err = self._packet.read4ByteTxRx(self._port, self._id, addr)
        self._check(res, err, f"read address {addr}")
        return value

    def write1(self, addr: int, value: int) -> None:
        res, err = self._packet.write1ByteTxRx(self._port, self._id, addr, value)
        self._check(res, err, f"write address {addr}")

    def write2(self, addr: int, value: int) -> None:
        res, err = self._packet.write2ByteTxRx(self._port, self._id, addr, value)
        self._check(res, err, f"write address {addr}")

    def write4(self, addr: int, value: int) -> None:
        res, err = self._packet.write4ByteTxRx(self._port, self._id, addr, value)
        self._check(res, err, f"write address {addr}")


def to_signed(value: int, bits: int) -> int:
    if value >= 1 << (bits - 1):
        value -= 1 << bits
    return value


def run_spring(args: Args) -> None:
    if args.goal_current > args.max_current:
        raise ValueError(
            f"goal_current {args.goal_current} mA is above max_current {args.max_current} mA"
        )

    port_handler = PortHandler(args.port)
    packet_handler = PacketHandler(2.0)
    if not port_handler.openPort():
        raise RuntimeError("Failed to open the port")
    if not port_handler.setBaudRate(args.baud):
        raise RuntimeError(f"Failed to change the baudrate, {args.baud}")

    motor = Motor(port_handler, packet_handler, args.gripper_id)
    original_mode = None
    try:
        original_mode = motor.read1(ADDR_OPERATING_MODE)
        current_limit = motor.read2(ADDR_CURRENT_LIMIT)
        start_pos = motor.read4(ADDR_PRESENT_POSITION)
        print("original operating mode : ", original_mode)
        print("current limit (mA)      : ", current_limit)
        print("start position (deg)    : ", f"{start_pos * 360 / 4096:.1f}")

        # operating mode can only be changed with torque off
        motor.write1(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        motor.write1(ADDR_OPERATING_MODE, CURRENT_BASED_POSITION_MODE)
        motor.write2(ADDR_GOAL_CURRENT, min(args.goal_current, current_limit))
        motor.write4(ADDR_GOAL_POSITION, int(round(args.open_deg * 4096 / 360)))
        motor.write1(ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        print()
        print(f"spring on, {args.goal_current} mA, Ctrl+C to stop")

        end_time = time.time() + args.duration
        while time.time() < end_time:
            pos = motor.read4(ADDR_PRESENT_POSITION) * 360 / 4096
            cur = to_signed(motor.read2(ADDR_PRESENT_CURRENT), 16)
            print(f"\rpos {pos:6.1f} deg   current {cur:5d} mA   ", end="")
            time.sleep(0.05)
        print()
    except KeyboardInterrupt:
        print()
    finally:
        # always leave the motor as it was found
        try:
            motor.write1(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if original_mode is not None:
                motor.write1(ADDR_OPERATING_MODE, original_mode)
                print("torque off, operating mode restored to ", original_mode)
        finally:
            port_handler.closePort()


def main(args: Args) -> None:
    run_spring(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
