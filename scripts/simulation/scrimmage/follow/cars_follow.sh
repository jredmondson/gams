#/bin/bash

PLATFORM="scrimmage"
echo gams_controller -mc 5 -n 5 -nt -p $PLATFORM -M $GAMS_ROOT/scripts/simulation/scrimmage/follow/follow.mf --gams-level 10
gams_controller -mc 5 -n 5 -nt -p $PLATFORM -M $GAMS_ROOT/scripts/simulation/scrimmage/follow/cars_follow.mf --gams-level 10 
