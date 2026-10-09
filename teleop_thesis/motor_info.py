import csv
import os
from dataclasses import dataclass
from datetime import datetime
from typing import Tuple

import tyro
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.robotis_def import COMM_SUCCESS

# Constants (address, length)
ADDR_MODEL_NUMBER = (0, 2)
ADDR_FIRMWARE_VERSION = (6, 1)
ADDR_BAUD_RATE = (8, 1)
ADDR_RETURN_DELAY_TIME = (9, 1)
ADDR_OPERATING_MODE = (11, 1)
ADDR_CURRENT_LIMIT = (38, 2)
ADDR_TORQUE_ENABLE = (64, 1)
ADDR_PRESENT_INPUT_VOLTAGE = (144, 2)
ADDR_PRESENT_TEMPERATURE = (146, 1)

MODEL_NAMES = {
    1020: "XM430-W350",
    1030: "XM430-W210",
    1060: "XL430-W250",
    1190: "XL330-M077",
    1200: "XL330-M288",
    1210: "XC330-T181",
    1220: "XC330-T288",
}

BAUD_RATES = {0: 9600, 1: 57600, 2: 115200, 3: 1000000, 4: 2000000, 5: 3000000, 6: 4000000}


@dataclass
class Args:
    port: str = (
        "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAK8B0I-if00-port0"
    )
    """The port that GELLO is connected to."""

    baud: int = 57600
    """The baudrate of the dynamixel bus."""

    ids: Tuple[int, ...] = (1, 2, 3, 4, 5, 6, 7)
    """The dynamixel ids to read, gripper included."""

    outdir: str = "teleop_thesis/data"
    """Where the table is saved."""


def read(packet_handler, port_handler, dxl_id: int, addr: Tuple[int, int]) -> int:
    address, length = addr
    if length == 1:
        value, dxl_comm_result, dxl_error = packet_handler.read1ByteTxRx(
            port_handler, dxl_id, address
        )
    else:
        value, dxl_comm_result, dxl_error = packet_handler.read2ByteTxRx(
            port_handler, dxl_id, address
        )
    if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
        raise RuntimeError(f"Failed to read address {address} for Dynamixel with ID {dxl_id}")
    return value


def get_info(args: Args) -> None:
    port_handler = PortHandler(args.port)
    packet_handler = PacketHandler(2.0)
    if not port_handler.openPort():
        raise RuntimeError("Failed to open the port")
    if not port_handler.setBaudRate(args.baud):
        raise RuntimeError(f"Failed to change the baudrate, {args.baud}")

    header = [
        "id",
        "model",
        "firmware",
        "baud",
        "return_delay_us",
        "operating_mode",
        "current_limit_raw",
        "torque_enabled",
        "voltage_V",
        "temp_C",
    ]
    rows = []
    try:
        # read only, nothing is written to the motors
        for dxl_id in args.ids:
            model = read(packet_handler, port_handler, dxl_id, ADDR_MODEL_NUMBER)
            baud = read(packet_handler, port_handler, dxl_id, ADDR_BAUD_RATE)
            rows.append(
                [
                    dxl_id,
                    MODEL_NAMES.get(model, f"unknown ({model})"),
                    read(packet_handler, port_handler, dxl_id, ADDR_FIRMWARE_VERSION),
                    BAUD_RATES.get(baud, f"unknown ({baud})"),
                    # unit is 2 microseconds
                    2 * read(packet_handler, port_handler, dxl_id, ADDR_RETURN_DELAY_TIME),
                    read(packet_handler, port_handler, dxl_id, ADDR_OPERATING_MODE),
                    read(packet_handler, port_handler, dxl_id, ADDR_CURRENT_LIMIT),
                    read(packet_handler, port_handler, dxl_id, ADDR_TORQUE_ENABLE),
                    # unit is 0.1 V
                    read(packet_handler, port_handler, dxl_id, ADDR_PRESENT_INPUT_VOLTAGE) / 10,
                    read(packet_handler, port_handler, dxl_id, ADDR_PRESENT_TEMPERATURE),
                ]
            )
    finally:
        port_handler.closePort()

    print("  ".join(header))
    for row in rows:
        print("  ".join(str(x) for x in row))

    os.makedirs(args.outdir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(args.outdir, f"motor_info_{stamp}.csv")
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)
    print()
    print("saved to: ", path)


def main(args: Args) -> None:
    get_info(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
