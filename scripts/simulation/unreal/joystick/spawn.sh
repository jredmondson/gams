#/bin/bash

TYPE="quadcopter"
ID=0
FILE="init0.mf"
SCRIPTS_DIR=`dirname $0`

if [ $# -ge 1 ]; then
  if [ "$1" == "help" ] || [ "$1" == "-h" ]; then
    echo "$0 [scriptfile] [agent id]"
    exit 0
  fi

  FILE="$1"

  if [ $# -ge 2 ]; then
    ID=$2
  fi
fi

SCRIPT="$SCRIPTS_DIR/$FILE"

echo gams_controller -i $ID -p osc-joystick -M $SCRIPT -A null -z 30 --gams-level 6
gams_controller -i $ID -p osc-joystick -M $SCRIPT -A null -z 30 --gams-level 6

  
exit 0
