"""Registers this leader arm's calibration, then runs upstream run_env.py unchanged."""
import runpy
import sys

from leader_config import register


# default to the real leader arm, upstream's default agent is a dummy that reads all zeros
if not any(a == "--agent" or a.startswith("--agent=") for a in sys.argv[1:]):
    sys.argv.append("--agent=gello")

register()
runpy.run_path("experiments/run_env.py", run_name="__main__")
