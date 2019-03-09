#/bin/bash

TYPE="quadcopter"
N=5
NT=0
SCRIPTS_DIR=`dirname $0`
SCRIPT="$SCRIPTS_DIR/wing.mf"

if [ $# -ge 1 ]; then
  if [ "$1" == "help" ] || [ "$1" == "-h" ]; then
    echo "$0 [type] [num agents] [no-transport]"
    exit 0
  fi

  TYPE="$1"

  if [ $# -ge 2 ]; then
    N=$2
  fi

  if [ $# -ge 3 ]; then
    NT=1
  fi
fi

if [ $NT -eq 1 ]; then
  echo gams_controller -mc $N -n $N -nt -p osc-$TYPE -M $SCRIPT -z 2
  gams_controller -mc $N -n $N -nt -p osc-$TYPE -M $SCRIPT -z 2 
else
  echo gams_controller -mc $N -n $N -p osc-$TYPE -M $SCRIPT -z 2 
  gams_controller -mc $N -n $N -p osc-$TYPE -M $SCRIPT -z 2 
fi
  
exit 0
