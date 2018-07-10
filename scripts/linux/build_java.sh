#!/bin/bash
# Build the required libraries for GAMS with java

if [ -z $GAMS_ROOT ] ; then
  export SCRIPTS_DIR=`dirname $0`
else
  export SCRIPTS_DIR=$GAMS_ROOT/scripts/linux
fi

echo "Building madara gams java $@"
echo "using GAMS_ROOT=$GAMS_ROOT"
echo "Using SCRIPTS_DIR=$SCRIPTS_DIR"

$SCRIPTS_DIR/base_build.sh java madara gams $@
