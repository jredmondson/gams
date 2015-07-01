#!/bin/bash
# Build the required libraries for GAMS
#
# There are several expected environment variables
#   $CORES        - number of build jobs to launch with make
#   $ACE_ROOT     - location of local copy of ACE subversion repository from
#                   svn://svn.dre.vanderbilt.edu/DOC/Middleware/sets-anon/ACE
#   $MADARA_ROOT  - location of local copy of MADARA git repository from
#                   http://madara.googlecode.com/svn/trunk/
#   $GAMS_ROOT    - location of this GAMS git repository
#   $VREP_ROOT    - location of VREP installation, if applicable
#
# For android
#   $NDK_BIN      - This should be the path to the Android NDK binaries for the
#                   platform you are trying to deploy (e.g. the arm toolchain)
#   $VREP_ROOT    - location of VREP installation, if applicable
#   $LOCAL_CROSS_PREFIX
#                 - Set this to the toolchain prefix
#
# For java
#   $JAVA_HOME

TESTS=0
VREP=0
JAVA=0
ROS=0
ANDROID=0

for var in "$@"
do
  if [ "$var" = "tests" ]; then
    TESTS=1
  elif [ "$var" = "vrep" ]; then
    VREP=1
  elif [ "$var" = "java" ]; then
    JAVA=1
  elif [ "$var" = "ros" ]; then
    ROS=1
  elif [ "$var" = "android" ]; then
    ANDROID=1
    JAVA=1
  fi
done

# echo build information
echo "Using $CORES build jobs"
echo "MADARA will be built from $MADARA_ROOT"
echo "ACE will be built from $ACE_ROOT"
echo "GAMS will be built from $GAMS_ROOT"
echo "TESTS has been set to $TESTS"
echo "VREP has been set to $VREP"
echo "ROS has been set to $ROS"
echo "ANDROID has been set to $ANDROID"
echo "JAVA has been set to $JAVA"

if [ $JAVA -eq 1 ]; then
  echo "JAVA_HOME is referencing $JAVA_HOME"
fi

if [ $VREP -eq 1 ]; then
  echo "VREP_ROOT is referencing $VREP_ROOT"
fi

if [ $ANDROID -eq 1 ]; then
  echo "CROSS_COMPILE is set to $LOCAL_CROSS_PREFIX"
fi

echo ""

# build ACE, all build information (compiler and options) will be set here
echo "Building ACE"
if [ $android = 1 ]; then
  # use the android specific files, we use custom config file for android due to build bug in ACE
  echo "#include \"$GAMS_ROOT/scripts/linux/config-android.h\"" > $ACE_ROOT/ace/config.h

  # Android does not support versioned libraries and requires cross-compiling
  echo -e "versioned_so=0\nCROSS_COMPILE=$(LOCAL_CROSS_PREFIX)\ninclude \$(ACE_ROOT)/include/makeinclude/platform_android.GNU" > $ACE_ROOT/include/makeinclude/platform_macros.GNU
else
  # use linux defaults
  echo "#include \"ace/config-linux.h\"" > $ACE_ROOT/ace/config.h
  echo "include \$(ACE_ROOT)/include/makeinclude/platform_linux.GNU" > $ACE_ROOT/include/makeinclude/platform_macros.GNU
fi
cd $ACE_ROOT/ace
perl $ACE_ROOT/bin/mwc.pl -type gnuace ace.mwc
make realclean -j $CORES
make -j $CORES

# build MADARA
echo "Building MADARA"
cd $MADARA_ROOT
perl $ACE_ROOT/bin/mwc.pl -type gnuace -features java=$JAVA,tests=$TESTS MADARA.mwc
make realclean -j $CORES
make java=$JAVA tests=$TESTS -j $CORES
if [ $JAVA -eq 1 ]; then
  # sometimes the jar'ing will occur before all classes are actually built when performing
  # multi-job builds, fix by deleting class files and recompiling with single build job
  find . -name "*.class" -delete
  make tests=$TESTS java=$JAVA
fi

# build GAMS
echo "Building GAMS"
cd $GAMS_ROOT
perl $ACE_ROOT/bin/mwc.pl -type gnuace -features java=$JAVA,ros=$ROS,vrep=$VREP,tests=$TESTS,android=$ANDROID gams.mwc
make realclean -j $CORES
make java=$JAVA ros=$ROS vrep=$VREP tests=$TESTS android=$ANDROID -j $CORES
if [ $JAVA -eq 1 ]; then
  # sometimes the jar'ing will occur before all classes are actually built when performing
  # multi-job builds, fix by deleting class files and recompiling with single build job
  find . -name "*.class" -delete
  make java=$JAVA ros=$ROS vrep=$VREP tests=$TESTS android=$ANDROID
fi
