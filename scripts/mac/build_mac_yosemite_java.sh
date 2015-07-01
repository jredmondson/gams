#!/bin/bash
# Build the required libraries for GAMS
#
# There are several expected environment variables
#   $CORES        - number of build jobs to launch with make
#   $ACE_ROOT     - location of local copy of ACE subversion repository from
#                   svn://svn.dre.vanderbilt.edu/DOC/Middleware/sets-anon/ACE
#                   Revision 97777 from Jun 6, 2014 is known to work.
#   $MADARA_ROOT  - location of local copy of MADARA git repository from
#                   http://madara.googlecode.com/svn/trunk/
#   $GAMS_ROOT    - location of this GAMS git repository
#   $VREP_ROOT    - location of VREP installation, if applicable

TESTS=0
VREP=0

echo "Arguments can be \"tests\" or \"vrep\" to enable these features"
echo "Arg 1 is $1"
echo "Arg 2 is $2"

if [ "$1" = "tests" -o "$2" = "tests" ]; then
  TESTS=1
fi

if [ "$1" = "vrep" -o "$2" = "vrep" ]; then
  VREP=1
fi

# echo build information
echo "Using $CORES build jobs"
echo "MADARA will be built from $MADARA_ROOT"
echo "ACE will be built from $ACE_ROOT"
echo "GAMS will be built from $GAMS_ROOT"
echo "TESTS has been set to $TESTS"
echo "VREP has been set to $VREP"

if [ $VREP -eq 1 ]; then
  echo "VREP_ROOT is referencing $VREP_ROOT"
fi

echo ""

# build ACE
echo "Building ACE"
echo "#include \"ace/config-macosx-yosemite.h\"" > $ACE_ROOT/ace/config.h
echo "include \$(ACE_ROOT)/include/makeinclude/platform_macosx_yosemite.GNU" > $ACE_ROOT/include/makeinclude/platform_macros.GNU
cd $ACE_ROOT/ace
make realclean -j $CORES
perl $ACE_ROOT/bin/mwc.pl -type make ACE.mwc
make realclean -j $CORES
make -j $CORES

# build MADARA
echo "Building MADARA"
cd $MADARA_ROOT
make realclean -j $CORES
find . -name "*.class" -delete
perl $ACE_ROOT/bin/mwc.pl -type make -features java=1,tests=$TESTS MADARA.mwc
make realclean -j $CORES
make java=1 tests=$TESTS -j $CORES
find . -name "*.class" -delete
make java=1 tests=$TESTS

# build GAMS
echo "Building GAMS"
cd $GAMS_ROOT
make realclean -j $CORES
find . -name "*.class" -delete
perl $ACE_ROOT/bin/mwc.pl -type make -features java=1,vrep=$VREP,tests=$TESTS gams.mwc
make realclean -j $CORES
make java=1 vrep=$VREP tests=$TESTS -j $CORES && ln -s $GAMS_ROOT/bin/gams_controller $GAMS_ROOT/gams_controller && ln -s $GAMS_ROOT/bin/dynamic_simulation $GAMS_ROOT/dynamic_simulation
find . -name "*.class" -delete
make java=1 vrep=$VREP tests=$TESTS
