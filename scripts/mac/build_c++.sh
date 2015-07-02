#!/bin/bash
# Build the required libraries for GAMS

if [ $# -gt 0 ]; then
  if [ "$1" = "lion" ]; then
  elif [ "$1" = "mountainlion" ]; then
  elif [ "$1" = "mavericks" ]; then
  elif [ "$1" = "yosemite" ]; then
  else
    echo "Invalid argument"
    echo "  Usage: $0 <mac_version>"
    echo "  Build the C++ libraries for GAMS"
    echo "  <mac_version>   select version of mac to build for, if missing then auto-detect"
    echo "                  lion"
    echo "                  mountainlion"
    echo "                  mavericks"
    echo "                  yosemite"
    echo ""
    echo "The following environment variables are used"
    echo "CORES        - number of build jobs to launch with make, optional"
    echo "ACE_ROOT     - location of local copy of ACE subversion repository from"
    echo "               svn://svn.dre.vanderbilt.edu/DOC/Middleware/sets-anon/ACE"
    echo "MADARA_ROOT  - location of local copy of MADARA git repository from"
    echo "               git://git.code.sf.net/p/madara/code"
    echo "GAMS_ROOT    - location of this GAMS git repository"
    echo "VREP_ROOT    - location of VREP installation"
    exit
  fi
fi

$GAMS_ROOT/scripts/mac/base_build.sh vrep $1
