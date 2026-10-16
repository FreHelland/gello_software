from dynamixel_sdk.robotis_def import COMM_SUCCESS

from gello.robots.dynamixel import DynamixelRobot
from leader_config import GELLO_CONFIG

# Constants (XL330 control table)
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_P_GAIN = 84
ADDR_GOAL_CURRENT = 102
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
CURRENT_BASED_POSITION_MODE = 5

GRIPPER_ID = GELLO_CONFIG.gripper_config[0]
OPEN_DEG = 190.0
HARD_MAX_CURRENT_MA = 1000

_state = {"driver": None, "original_mode": None, "current": None}


def _check(result, error, what: str) -> None:
    if result != COMM_SUCCESS or error != 0:
        raise RuntimeError(f"Failed to {what} for Dynamixel with ID {GRIPPER_ID}")


def enable_spring(driver, current_ma: int) -> None:
    current_ma = int(min(current_ma, HARD_MAX_CURRENT_MA))
    port, packet = driver._portHandler, driver._packetHandler
    # the driver's reading thread shares the port, so every access holds its lock
    with driver._lock:
        mode, res, err = packet.read1ByteTxRx(port, GRIPPER_ID, ADDR_OPERATING_MODE)
        _check(res, err, "read operating mode")
        _state["driver"], _state["original_mode"] = driver, mode
        _check(*packet.write1ByteTxRx(port, GRIPPER_ID, ADDR_TORQUE_ENABLE, 0), "disable torque")
        _check(*packet.write1ByteTxRx(port, GRIPPER_ID, ADDR_OPERATING_MODE, CURRENT_BASED_POSITION_MODE), "set operating mode")
        _check(*packet.write2ByteTxRx(port, GRIPPER_ID, ADDR_GOAL_CURRENT, current_ma), "set goal current")
        _check(*packet.write4ByteTxRx(port, GRIPPER_ID, ADDR_GOAL_POSITION, int(round(OPEN_DEG * 4096 / 360))), "set goal position")
        _check(*packet.write1ByteTxRx(port, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1), "enable torque")
    _state["current"] = current_ma
    print(f"trigger spring on, {current_ma} mA (original mode {mode})")


def set_current(current_ma: int) -> None:
    driver = _state["driver"]
    if driver is None:
        return
    current_ma = int(min(max(current_ma, 0), HARD_MAX_CURRENT_MA))
    with driver._lock:
        _check(
            *driver._packetHandler.write2ByteTxRx(
                driver._portHandler, GRIPPER_ID, ADDR_GOAL_CURRENT, current_ma
            ),
            "set goal current",
        )
    _state["current"] = current_ma


def set_goal_deg(deg: float) -> None:
    driver = _state["driver"]
    if driver is None:
        return
    with driver._lock:
        _check(
            *driver._packetHandler.write4ByteTxRx(
                driver._portHandler, GRIPPER_ID, ADDR_GOAL_POSITION, int(round(deg * 4096 / 360))
            ),
            "set goal position",
        )


def _get_gain(addr: int, name: str) -> int:
    driver = _state["driver"]
    with driver._lock:
        value, res, err = driver._packetHandler.read2ByteTxRx(driver._portHandler, GRIPPER_ID, addr)
    _check(res, err, f"read {name}")
    return value


def _set_gain(addr: int, name: str, gain: int) -> None:
    # RAM setting, it also resets to the motor's default on power cycle
    driver = _state["driver"]
    if driver is None:
        return
    gain = int(min(max(gain, 0), 16383))
    with driver._lock:
        _check(
            *driver._packetHandler.write2ByteTxRx(driver._portHandler, GRIPPER_ID, addr, gain),
            f"set {name}",
        )


def get_p_gain() -> int:
    return _get_gain(ADDR_POSITION_P_GAIN, "position P gain")


def set_p_gain(gain: int) -> None:
    _set_gain(ADDR_POSITION_P_GAIN, "position P gain", gain)


def get_d_gain() -> int:
    return _get_gain(ADDR_POSITION_D_GAIN, "position D gain")


def set_d_gain(gain: int) -> None:
    _set_gain(ADDR_POSITION_D_GAIN, "position D gain", gain)


def read_present_current() -> int:
    driver = _state["driver"]
    if driver is None:
        return 0
    with driver._lock:
        value, res, err = driver._packetHandler.read2ByteTxRx(
            driver._portHandler, GRIPPER_ID, ADDR_PRESENT_CURRENT
        )
    _check(res, err, "read present current")
    # signed 16 bit, unit is 1 mA
    return value - 65536 if value >= 32768 else value


def get_current() -> int:
    return _state["current"]


def disable_spring() -> None:
    driver, mode = _state["driver"], _state["original_mode"]
    if driver is None:
        return
    gains = _state.pop("original_gains", None)
    if gains is not None:
        set_p_gain(gains[0])
        set_d_gain(gains[1])
    port, packet = driver._portHandler, driver._packetHandler
    with driver._lock:
        packet.write1ByteTxRx(port, GRIPPER_ID, ADDR_TORQUE_ENABLE, 0)
        if mode is not None:
            packet.write1ByteTxRx(port, GRIPPER_ID, ADDR_OPERATING_MODE, mode)
    _state["driver"] = None
    print(f"trigger spring off, operating mode restored to {mode}")


def install_spring(current_ma: int, p_gain: int = 0, d_gain: int = 0) -> None:
    """Turns the trigger motor on as soon as upstream has built the leader arm. Gains of 0 keep the motor's own."""
    original_init = DynamixelRobot.__init__

    def init_with_spring(self, *args, **kwargs):
        # upstream disables torque on every motor at the end of __init__, so the spring goes on after it
        original_init(self, *args, **kwargs)
        driver = self._driver
        if getattr(driver, "_is_fake", True) or GRIPPER_ID not in driver._ids:
            print("no real gripper motor found, trigger spring not enabled")
            return
        enable_spring(driver, current_ma)
        _state["original_gains"] = (get_p_gain(), get_d_gain())
        if p_gain > 0:
            set_p_gain(p_gain)
        if d_gain > 0:
            set_d_gain(d_gain)
        print(f"trigger gains P {get_p_gain()} D {get_d_gain()} (motor's own P {_state['original_gains'][0]} D {_state['original_gains'][1]})")

    DynamixelRobot.__init__ = init_with_spring
