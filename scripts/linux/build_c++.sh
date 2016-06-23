#!/bin/bash
# Build the required libraries for GAMS

if [ -z $GAMS_ROOT ] ; then
  export SCRIPTS_DIR=`dirname $0`
else
  export SCRIPTS_DIR=$GAMS_ROOT/scripts/linux 
fi

echo "Building ace madara gams vrep $@"
echo "using GAMS_ROOT=$GAMS_ROOT"
echo "Using SCRIPTS_DIR=$SCRIPTS_DIR"
$SCRIPTS_DIR/base_build.sh vrep vrep-config ace madara gams $@
