#/bin/bash

N=18
NT=0
SCRIPTS_DIR=`dirname $0`
SCRIPT="$SCRIPTS_DIR/galois.mf"

if [ $# -ge 1 ]; then
  if [ "$1" == "help" ] || [ "$1" == "-h" ]; then
    echo "$0 [num agents] [no-transport]"
    exit 0
  fi

  if [ $# == 1 ]; then
    NT=1
  fi
fi

if [ $NT -eq 1 ]; then
  echo gams_controller -mc $N -n $N -nt -p osc -M $SCRIPT -z 4
  gams_controller -mc $N -n $N -nt -p osc -M $SCRIPT -z 4 
else
  echo gams_controller -mc $N -n $N -p osc -M $SCRIPT -z 4
  gams_controller -mc $N -n $N -p osc -M $SCRIPT -z 4
fi
  
exit 0
