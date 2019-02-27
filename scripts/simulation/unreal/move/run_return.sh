#/bin/bash

TYPE="quadcopter"
N=1
NT=0
SCRIPTS_DIR=`dirname $0`
SCRIPT="$SCRIPTS_DIR/origin.mf"

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
  echo gams_controller -mc $N -nt -p osc-$TYPE -M $SCRIPT -z 10
  gams_controller -mc $N -nt -p osc-$TYPE -M $SCRIPT -z 10
else
  echo gams_controller -mc $N -n $N -p osc-$TYPE -M $SCRIPT -z 10
  gams_controller -mc $N -n $N -p osc-$TYPE -M $SCRIPT -z 10
fi
  
exit 0
