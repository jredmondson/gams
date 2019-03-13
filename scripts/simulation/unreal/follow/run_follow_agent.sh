#/bin/bash

TYPE="quadcopter"
LEAD="agent.0"
N=10
NT=0
ID=1
SCRIPTS_DIR=`dirname $0`
SCRIPT="$SCRIPTS_DIR/follow_agent.mf"
LOG_LEVEL=4

if [ $# -ge 1 ]; then
  if [ "$1" == "help" ] || [ "$1" == "-h" ]; then
    echo "$0 [leader=agent.0] [id start=1] [type=quad] [num agents=10] [no-transport]"
    exit 0
  fi

  LEAD="$1"

  if [ $# -ge 2 ]; then
    ID=$2
  fi

  if [ $# -ge 3 ]; then
    TYPE=1
  fi

  if [ $# -ge 4 ]; then
    N=1
  fi

  if [ $# -ge 5 ]; then
    NT=1
  fi
fi

if [ $NT -eq 1 ]; then
  echo gams_controller -i $ID -mc $N -n $N -nt -p osc-$TYPE -0 ".lead='$LEAD'" -M $SCRIPT -z 4 --gams-level $LOG_LEVEL
  gams_controller -i $ID -mc $N -n $N -nt -p osc-$TYPE -0 ".lead='$LEAD'" -M $SCRIPT -z 4 --gams-level $LOG_LEVEL
else
  echo gams_controller -i $ID -mc $N -n $N -p osc-$TYPE -0 ".lead='$LEAD'" -M $SCRIPT -z 4 --gams-level $LOG_LEVEL
  gams_controller -i $ID -mc $N -n $N -p osc-$TYPE -0 ".lead='$LEAD'" -M $SCRIPT -z 4 --gams-level $LOG_LEVEL
fi
  
exit 0
