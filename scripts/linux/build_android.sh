#!/bin/bash
# Build the required libraries for GAMS on android

$GAMS_ROOT/scripts/linux/base_build.sh android $@

$GAMS_ROOT/scripts/linux/copy_libs.sh
