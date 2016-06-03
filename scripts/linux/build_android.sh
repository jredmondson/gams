#!/bin/bash
# Build the required libraries for GAMS on android

if [ -z $GAMS_ROOT ] ; then
  export SCRIPTS_DIR=`dirname $0`
else
  export SCRIPTS_DIR=$GAMS_ROOT/scripts/linux
fi

echo "Building ace madara gams android $@"
echo "using GAMS_ROOT=$GAMS_ROOT"
echo "Using SCRIPTS_DIR=$SCRIPTS_DIR"

$SCRIPTS_DIR/base_build.sh ace madara gams android $@

$SCRIPTS_DIR/copy_libs.sh
