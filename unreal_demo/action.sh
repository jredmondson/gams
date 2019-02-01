#!/bin/bash
#
# Copyright (c) 2015-2018 Carnegie Mellon University. All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following acknowledgments
# and disclaimers.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# 3. The names "Carnegie Mellon University," "SEI" and/or "Software
# Engineering Institute" shall not be used to endorse or promote
# products derived from this software without prior written
# permission. For written permission, please contact
# permission@sei.cmu.edu.
#
# 4. Products derived from this software may not be called "SEI" nor
# may "SEI" appear in their names without prior written permission of
# permission@sei.cmu.edu.
#
# 5. Redistributions of any form whatsoever must retain the following
# acknowledgment:
#
# Copyright 2015-2018 Carnegie Mellon University
#
# This material is based upon work funded and supported by the
# Department of Defense under Contract No. FA8721-05-C-0003 with
# Carnegie Mellon University for the operation of the Software
# Engineering Institute, a federally funded research and development
# center.
#
# Any opinions, findings and conclusions or recommendations expressed
# in this material are those of the author(s) and do not necessarily
# reflect the views of the United States Department of Defense.
#
# NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE
# ENGINEERING INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS"
# BASIS. CARNEGIE MELLON UNIVERSITY MAKES NO WARRANTIES OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, AS TO ANY MATTER INCLUDING, BUT NOT
# LIMITED TO, WARRANTY OF FITNESS FOR PURPOSE OR MERCHANTABILITY,
# EXCLUSIVITY, OR RESULTS OBTAINED FROM USE OF THE MATERIAL. CARNEGIE
# MELLON UNIVERSITY DOES NOT MAKE ANY WARRANTY OF ANY KIND WITH
# RESPECT TO FREEDOM FROM PATENT, TRADEMARK, OR COPYRIGHT
# INFRINGEMENT.
#
# This material has been approved for public release and unlimited
# distribution.
#
# DM-0002489
#
# Build a custom project created by gpc.pl


COMPILE=0
COMPILE_VREP=0
DOCS=0
PREREQS=0
RUN=0
VERBOSE=0
VREP=0
INSTALL_DIR=`pwd`
SCRIPTS_DIR=`dirname $0`

# just in case we're in git commits and empty directories were discarded
[ -d $SCRIPTS_DIR/src/algorithms ] || mkdir $SCRIPTS_DIR/src/algorithms
[ -d $SCRIPTS_DIR/src/algorithms/threads ] || mkdir $SCRIPTS_DIR/src/algorithms/threads
[ -d $SCRIPTS_DIR/src/containers ] || mkdir $SCRIPTS_DIR/src/containers
[ -d $SCRIPTS_DIR/src/filters ] || mkdir $SCRIPTS_DIR/src/filters
[ -d $SCRIPTS_DIR/src/platforms ] || mkdir $SCRIPTS_DIR/src/platforms
[ -d $SCRIPTS_DIR/src/platforms/threads ] || mkdir $SCRIPTS_DIR/src/platforms/threads
[ -d $SCRIPTS_DIR/src/threads ] || mkdir $SCRIPTS_DIR/src/threads
[ -d $SCRIPTS_DIR/src/transports ] || mkdir $SCRIPTS_DIR/src/transports

if [ -z $CORES ] ; then
  echo "CORES unset, so setting it to default of 1"
  echo "  If you have more than one CPU core, try export CORES=<num cores>"
  echo "  CORES=1 (the default) will be much slower than CORES=<num cores>"
  export CORES=1  
fi

for var in "$@"
do
  if [ "$var" = "compile" ]; then
    COMPILE=1
  elif [ "$var" = "compile-vrep" ]; then
    COMPILE_VREP=1
  elif [ "$var" = "docs" ]; then
    DOCS=1
  elif [ "$var" = "prereqs" ]; then
    PREREQS=1
  elif [ "$var" = "run" ]; then
    RUN=1
  elif [ "$var" = "sim" ]; then
    RUN=1
  elif [ "$var" = "verbose" ]; then
    VERBOSE=1
  elif [ "$var" = "vrep" ]; then
    VREP=1
  else
    echo "Invalid argument: $var"
    echo "  args can be zero or more of the following, space delimited"
    echo "  compile         build the custom project"
    echo "  compile-vrep    compile with vrep support"
    echo "  prereqs         apt-get doxygen and other prereqs"
    echo "  run|sim         run the simulation"
    echo "  verbose         print verbose information during this script"
    echo "  vrep            start vrep before running the simulation"
    echo "  help            get script usage"
    echo ""
    echo "The following environment variables are used"
    echo "  CORES               - number of build jobs to launch with make, optional"
    echo "  MPC_ROOT            - location of MPC"
    echo "  MADARA_ROOT         - location of local copy of MADARA git repository from"
    echo "                        git://git.code.sf.net/p/madara/code"
    echo "  GAMS_ROOT           - location of this GAMS git repository"
    echo "  VREP_ROOT           - location of VREP installation"
    echo "  JAVA_HOME           - location of JDK"
    exit
  fi
done

if [ $PREREQS -eq 1 ]; then
  sudo apt-get install doxygen graphviz
fi

if [ $COMPILE -eq 1 ]; then

  if [ $VERBOSE -eq 1 ]; then
    echo "Generating project workspace..."
  fi

  cd $SCRIPTS_DIR
  $MPC_ROOT/mwc.pl -type make -features vrep=$COMPILE_VREP,tests=0,docs=1 workspace.mwc
  
  
  if [ $VERBOSE -eq 1 ]; then
    echo "Cleaning project..."
  fi

  make realclean


  if [ $VERBOSE -eq 1 ]; then
    echo "Building project..."
  fi

  make vrep=$COMPILE_VREP docs=$DOCS -j $CORES
  
fi
  
if [ $VREP -eq 1 ]; then
  if [ $VERBOSE -eq 1 ]; then
    echo "Launching vrep..."
  fi

  $VREP_ROOT/vrep &
  
  if [ $VERBOSE -eq 1 ]; then
    echo "Sleeping for 10s to allow VREP to load..."
  fi

  sleep 10
fi

if [ $RUN -eq 1 ]; then
  if [ $VERBOSE -eq 1 ]; then
    echo "Running simulation..."
  fi

  cd $SCRIPTS_DIR/sim
  perl run.pl
fi

echo "Script finished"


