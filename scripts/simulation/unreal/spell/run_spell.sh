#/bin/bash

NT=0
SCRIPTS_DIR=`dirname $0`
SCRIPT="$SCRIPTS_DIR/spell.mf"
TEXT="hello"

if [ $# -ge 1 ]; then
  if [ "$1" == "help" ] || [ "$1" == "-h" ]; then
    echo "$0 [text='hello'] [no-transport]"
    exit 0
  else
    TEXT=$1
  fi

  if [ $# == 2 ]; then
    NT=1
  fi
fi

LETTERS=${#TEXT}
N=$(( LETTERS * 3 ))

if [ $NT -eq 1 ]; then
  echo gams_controller -mc $N -n $N -nt -p osc -0 ".text='$TEXT'" -M $SCRIPT -z 4
  gams_controller -mc $N -n $N -nt -p osc -0 ".text='$TEXT'" -M $SCRIPT -z 4 
else
  echo gams_controller -mc $N -n $N -p osc -0 ".text='$TEXT'" -M $SCRIPT -z 4
  gams_controller -mc $N -n $N -p osc -0 ".text='$TEXT'" -M $SCRIPT -z 4
fi
  
exit 0
