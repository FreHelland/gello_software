"""Same as run_gello.py, with the trigger spring on during teleop."""
import runpy
import sys

from leader_config import register
from leader_trigger import disable_spring, install_spring

GOAL_CURRENT_MA = 150

# default to the real leader arm, upstream's default agent is a dummy that reads all zeros
if not any(a == "--agent" or a.startswith("--agent=") for a in sys.argv[1:]):
    sys.argv.append("--agent=gello")

install_spring(GOAL_CURRENT_MA)
register()
try:
    runpy.run_path("experiments/run_env.py", run_name="__main__")
finally:
    disable_spring()
