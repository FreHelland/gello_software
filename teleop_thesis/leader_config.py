import numpy as np
from gello.agents.gello_agent import PORT_CONFIG_MAP, DynamixelRobotConfig

PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAK8B0I-if00-port0"

GELLO_CONFIG = DynamixelRobotConfig(
    joint_ids=(1, 2, 3, 4, 5, 6),
    joint_offsets=(0*np.pi/2, 0*np.pi/2, 2*np.pi/2, 2*np.pi/2, 2*np.pi/2, 0*np.pi/2),
    joint_signs=(1, 1, -1, 1, 1, 1),
    gripper_config=(7, 193, 147),
)


def register() -> None:
    PORT_CONFIG_MAP[PORT] = GELLO_CONFIG
