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
#   $NDK_BIN      - This should be the path to the Android NDK binaries for the
#                   platform you are trying to deploy (e.g. the arm toolchain)
#   $VREP_ROOT    - location of VREP installation, if applicable
#   

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
echo "JAVA_HOME is referencing $JAVA_HOME"
echo "NDK_BIN is referencing $NDK_BIN"
echo "TESTS has been set to $TESTS"
echo "VREP has been set to $VREP"

if [ $VREP -eq 1 ]; then
  echo "VREP_ROOT is referencing $VREP_ROOT"
fi

echo ""

CORES=1

# build ACE
echo "Building ACE"
echo "#include \"$GAMS_ROOT/scripts/linux/config-android.h\"" > $ACE_ROOT/ace/config.h
echo -e "versioned_so=0\nCROSS_COMPILE=\$(ARM_BIN)/\$(LOCAL_CROSS_PREFIX)\ninclude \$(ACE_ROOT)/include/makeinclude/platform_android.GNU" > $ACE_ROOT/include/makeinclude/platform_macros.GNU
cd $ACE_ROOT/ace
mwc.pl -type gnuace ace.mwc
make realclean -j $CORES
make -j $CORES

# build MADARA
echo "Building MADARA"
cd $MADARA_ROOT
make realclean -j $CORES
find . -name "*.class" -delete
mwc.pl -type gnuace -features java=1,android=1,tests=0 MADARA.mwc
make realclean -j $CORES
find . -name "*.class" -delete
make java=1 android=1 tests=0 -j $CORES

# build GAMS
echo "Building GAMS"
cd $GAMS_ROOT
make realclean -j $CORES
find . -name "*.class" -type f -delete
mwc.pl -type gnuace -features java=1,android=1,vrep=0,tests=0 gams.mwc
make java=1 android=1 -j $CORES
