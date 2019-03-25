#/bin/bash

TYPE="quadcopter"
LEAD="agent.0"
N=10
FOLLOWERS=2
NT=0
TARGETS=1
SCRIPTS_DIR=`dirname $0`
SCRIPT="$SCRIPTS_DIR/greet_and_follow.mf"
LOG_LEVEL=4

if [ $# -ge 1 ]; then
  if [ "$1" == "help" ] || [ "$1" == "-h" ]; then
    echo "$0 [type=quad] [targets=1] [followers=2] [num agents=10] [no-transport]"
    exit 0
  fi

  TYPE="$1"

  if [ $# -ge 2 ]; then
    TARGETS=$2
  fi

  if [ $# -ge 3 ]; then
    FOLLOWERS=$3
  fi

  if [ $# -ge 4 ]; then
    N=$4
  fi

  if [ $# -ge 5 ]; then
    NT=1
  fi
fi

if [ $NT -eq 1 ]; then
  echo gams_controller -mc $N -n $N -nt -p osc-$TYPE -0 ".targets=$TARGETS;.followers=$FOLLOWERS" -M $SCRIPT -z 4 --gams-level $LOG_LEVEL
  gams_controller -mc $N -n $N -nt -p osc-$TYPE -0 ".targets=$TARGETS;.followers=$FOLLOWERS" -M $SCRIPT -z 4 --gams-level $LOG_LEVEL
else
  echo gams_controller -mc $N -n $N -p osc-$TYPE -0 ".targets=$TARGETS;.followers=$FOLLOWERS" -M $SCRIPT -z 4 --gams-level $LOG_LEVEL
  gams_controller -mc $N -n $N -p osc-$TYPE -0 ".targets=$TARGETS;.followers=$FOLLOWERS" -M $SCRIPT -z 4 --gams-level $LOG_LEVEL
fi
  
exit 0
