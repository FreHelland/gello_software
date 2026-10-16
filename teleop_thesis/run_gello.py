"""Main teleop entry: registers this leader arm's calibration, then runs upstream run_env.py.

Use with teleop_thesis/launch_robot.py. Own flags (removed before run_env.py sees the arguments):
    --no-haptics    plain position teleop, trigger motor left off
"""
import runpy
import sys

from leader_config import PORT, register

HAPTICS = "--no-haptics" not in sys.argv
sys.argv = [a for a in sys.argv if a != "--no-haptics"]

# default to the real leader arm, upstream's default agent is a dummy that reads all zeros
if not any(a == "--agent" or a.startswith("--agent=") for a in sys.argv[1:]):
    sys.argv.append("--agent=gello")
# name the leader's port, otherwise upstream takes the first USB serial device it finds (e.g. a phone)
if not any(a == "--gello-port" or a.startswith("--gello-port=") for a in sys.argv[1:]):
    sys.argv.append(f"--gello-port={PORT}")

if HAPTICS:
    import gripper_haptics

    gripper_haptics.install()

register()
try:
    runpy.run_path("experiments/run_env.py", run_name="__main__")
except KeyboardInterrupt:
    pass
finally:
    if HAPTICS:
        gripper_haptics.shutdown()
