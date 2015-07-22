#!/bin/bash
# Build the required libraries for GAMS on android

#my_dir="$(dirname "$0")"
#source "$my_dir/env_vars.sh"

$GAMS_ROOT/scripts/linux/base_build.sh android $@

$GAMS_ROOT/scripts/linux/copy_libs.sh
