#/bin/bash

PLATFORM="scrimmage"

# run_simcontrol_threaded=1 for simcontrol in its own thread (recommended)

echo gams_controller -mc 5 -n 5 -nt -p $PLATFORM -M $GAMS_ROOT/scripts/simulation/scrimmage/follow/follow.mf --gams-level 10
gams_controller -mc 4000 -n 4000 -nt -p $PLATFORM -M $GAMS_ROOT/scripts/simulation/scrimmage/follow/follow.mf --gams-level 10 -0 "run_simcontrol_threaded=1"
