#!/bin/bash
# Build the required libraries for GAMS with java

if [ $# -gt 0 ]; then
  echo "Usage: $0"
  echo "Build the Java JAR for GAMS"
  echo ""
  echo "The following environment variables are used"
  echo "CORES               - number of build jobs to launch with make, optional"
  echo "ACE_ROOT            - location of local copy of ACE subversion repository from"
  echo "                      svn://svn.dre.vanderbilt.edu/DOC/Middleware/sets-anon/ACE"
  echo "MADARA_ROOT         - location of local copy of MADARA git repository from"
  echo "                      git://git.code.sf.net/p/madara/code"
  echo "GAMS_ROOT           - location of this GAMS git repository"
  echo "VREP_ROOT           - location of VREP installation"
  echo "JAVA_HOME           - location of JDK"
  exit
fi

$GAMS_ROOT/scripts/linux/base_build.sh java vrep
