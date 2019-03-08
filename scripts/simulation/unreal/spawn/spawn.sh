#/bin/bash

TYPE="quadcopter"
I=0
N=1
NT=0
SCRIPTS_DIR=`dirname $0`
SCRIPT="$SCRIPTS_DIR/init.mf"

if [ $# -ge 1 ]; then
  if [ "$1" == "help" ] || [ "$1" == "-h" ]; then
    echo "$0 [type=quad] [num agents=1] [starting id=0] [no-transport=false]"
    exit 0
  fi

  TYPE="$1"

  if [ $# -ge 2 ]; then
    N=$2
  fi

  if [ $# -ge 3 ]; then
    I=$3
  fi

  if [ $# -ge 3 ]; then
    NT=1
  fi
fi

if [ $NT -eq 1 ]; then
  echo gams_controller -i $I -mc $N -nt -p osc-$TYPE -M $SCRIPT -A null -z 2
  gams_controller -i $I -mc $N -nt -p osc-$TYPE -M $SCRIPT -A null -z 2
else
  echo gams_controller -i $I -mc $N -n $N -p osc-$TYPE -M $SCRIPT -A null -z 2
  gams_controller -i $I -mc $N -n $N -p osc-$TYPE -M $SCRIPT -A null -z 2
fi
  
exit 0
