#/bin/bash

N=1
NT=0
SCRIPTS_DIR=`dirname $0`
SCRIPT="$SCRIPTS_DIR/galois.mf"

if [ $# -ge 1 ]; then
  if [ "$1" == "help" ] || [ "$1" == "-h" ]; then
    echo "$0 [num agents] [no-transport]"
    exit 0
  fi

  N=$1

  if [ $# == 2 ]; then
    NT=1
  fi
fi

if [ $NT -eq 1 ]; then
  echo gams_controller -mc $N -nt -p osc -M $SCRIPT
  gams_controller -mc $N -nt -p osc -M $SCRIPT
else
  echo gams_controller -mc $N -n $N -p osc -M $SCRIPT
  gams_controller -mc $N -n $N -p osc -M $SCRIPT
fi
  
exit 0
