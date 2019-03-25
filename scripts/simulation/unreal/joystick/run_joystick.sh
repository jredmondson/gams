#/bin/bash

TYPE="quadcopter"
ID=0
N=1
FILE="init.mf"
SCRIPTS_DIR=`dirname $0`

if [ $# -ge 1 ]; then
  if [ "$1" == "help" ] || [ "$1" == "-h" ]; then
    echo "$0 [scriptfile=init.mf] [agent id=0] [num agent controllers=1]"
    exit 0
  fi

  FILE="$1"

  if [ $# -ge 2 ]; then
    ID=$2
  fi

  if [ $# -ge 3 ]; then
    N=$3
  fi

fi

SCRIPT="$SCRIPTS_DIR/$FILE"

echo gams_controller -mc $N -n $N -i $ID -p osc-joystick -M $SCRIPT -A null -z 5
gams_controller  -mc $N -n $N -i $ID -p osc-joystick -M $SCRIPT -A null -z 5

  
exit 0
