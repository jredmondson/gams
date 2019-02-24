#/bin/bash

N=1
NT=0
SCRIPTS_DIR=`dirname $0`

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
  gams_controller -mc $N -nt -p osc -M $SCRIPTS_DIR/20m_square.mf
else
  gams_controller -mc $N -p osc -M $SCRIPTS_DIR/20m_square.mf
fi
  
exit 0
