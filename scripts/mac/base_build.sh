#!/bin/bash
# Build the required libraries for GAMS
#
# There are several expected environment variables
#   $CORES        - number of build jobs to launch with make
#   $ACE_ROOT     - location of local copy of ACE subversion repository from
#                   svn://svn.dre.vanderbilt.edu/DOC/Middleware/sets-anon/ACE
#   $MADARA_ROOT  - location of local copy of MADARA git repository from
#                   git://git.code.sf.net/p/madara/code
#   $GAMS_ROOT    - location of this GAMS git repository
#   $VREP_ROOT    - location of VREP installation, if applicable

TESTS=0
VREP=0
JAVA=0
ANDROID=0
MAC_VERSION=0

# parse command line args
for var in "$@"
do
  if [ "$var" = "tests" ]; then
    TESTS=1
  elif [ "$var" = "vrep" ]; then
    VREP=1
  elif [ "$var" = "java" ]; then
    JAVA=1
  elif [ "$var" = "android" ]; then
    ANDROID=1
    JAVA=1
  elif [ "$var" = "lion" ]; then
    MAC_VERSION="lion"
  elif [ "$var" = "mountainlion" ]; then
    MAC_VERSION="mountainlion"
  elif [ "$var" = "mavericks" ]; then
    MAC_VERSION="mavericks"
  elif [ "$var" = "yosemite" ]; then
    MAC_VERSION="yosemite"
  else
    echo "Invalid argument: $var"
    echo "  Usage: $0 <args>"
    echo "  args can be zero or more of the following, space delimited"
    echo "  tests           build test executables"
    echo "  vrep            build with vrep support"
    echo "  java            build java jar"
    echo "  android         build android libs, turns on java"
    echo "  <mac_version>   select version of mac to build for, if missing then auto-detect"
    echo "                  lion"
    echo "                  mountainlion"
    echo "                  mavericks"
    echo "                  yosemite"
    exit
  fi
done

# auto-detect Mac version string
if [ $MAC_VERSION -eq 0 ]; then
  VER_STRING=`sw_vers -productVersion`
  if [ ${VER_STRING:0:4} = "10.7" ]; then
    echo "Auto-detected OS X Lion"
    MAC_VERSION="lion"
  elif [ ${VER_STRING:0:4} = "10.8" ]; then
    echo "Auto-detected OS X Mountain Lion"
    MAC_VERSION="mountainlion"
  elif [ ${VER_STRING:0:4} = "10.9" ]; then
    echo "Auto-detected OS X Mavericks"
    MAC_VERSION="mavericks"
  elif [ ${VER_STRING:0:5} = "10.10" ]; then
    echo "Auto-detected OS X Yosemite"
    MAC_VERSION="yosemite"
  else
    echo "Could not auto-detect Mac version"
    exit
  fi
fi

# auto-detect JDK
if [ $JAVA -eq 1 ]; then
  echo "Auto-detecting JDK"
  if [ $JAVA_HOME = "" ]; then
    JAVA_HOME=$(/usr/libexec/java_home -v 1.8)
  fi
  if [ $JAVA_HOME = "" ]; then
    JAVA_HOME=$(/usr/libexec/java_home -v 1.7)
  fi
  if [ $JAVA_HOME = "" ]; then
    JAVA_HOME=$(/usr/libexec/java_home -v 1.6)
  fi
  if [ $JAVA_HOME = "" ]; then
    echo "Failed to auto-detect JDK"
    exit
  fi
fi

# echo build information
echo "Using $CORES build jobs"
echo "MAC_VERSION is $MAC_VERSION"
echo "MADARA will be built from $MADARA_ROOT"
echo "ACE will be built from $ACE_ROOT"
echo "GAMS will be built from $GAMS_ROOT"
echo "TESTS has been set to $TESTS"
echo "VREP has been set to $VREP"
echo "ROS has been set to $ROS"
echo "ANDROID has been set to $ANDROID"
echo "JAVA has been set to $JAVA"

# output other env vars
if [ $JAVA -eq 1 ]; then
  echo "JAVA_HOME is referencing $JAVA_HOME"
fi

if [ $VREP -eq 1 ]; then
  echo "VREP_ROOT is referencing $VREP_ROOT"
fi

if [ $ANDROID -eq 1 ]; then
  echo "CROSS_COMPILE is set to $LOCAL_CROSS_PREFIX"
fi

# spacing
echo ""

# build ACE
echo "Building ACE"
if [ $ANDROID -eq 1 ]; then
  # use the android specific files, we use custom config file for android due to build bug in ACE
  echo "#include \"$GAMS_ROOT/scripts/linux/config-android.h\"" > $ACE_ROOT/ace/config.h

  # Android does not support versioned libraries and requires cross-compiling
  echo -e "versioned_so=0\nCROSS_COMPILE=$(LOCAL_CROSS_PREFIX)\ninclude \$(ACE_ROOT)/include/makeinclude/platform_android.GNU" > $ACE_ROOT/include/makeinclude/platform_macros.GNU
else
  # use mac defaults
  echo "#include \"ace/config-macosx-$MAC_VERSION.h\"" > $ACE_ROOT/ace/config.h
  echo "include \$(ACE_ROOT)/include/makeinclude/platform_macosx_$MAC_VERSION.GNU" > $ACE_ROOT/include/makeinclude/platform_macros.GNU
fi
cd $ACE_ROOT/ace
perl $ACE_ROOT/bin/mwc.pl -type make ACE.mwc
make realclean -j $CORES
make -j $CORES

# build MADARA
echo "Building MADARA"
cd $MADARA_ROOT
perl $ACE_ROOT/bin/mwc.pl -type make -features android=$ANDROID,java=$JAVA,tests=$TESTS MADARA.mwc
make realclean -j $CORES
make android=$ANDROID java=$JAVA tests=$TESTS -j $CORES

if [ $JAVA -eq 1 ]; then
  # sometimes the jar'ing will occur before all classes are actually built when performing
  # multi-job builds, fix by deleting class files and recompiling with single build job
  find . -name "*.class" -delete
  make android=$ANDROID java=$JAVA tests=$TESTS 
fi

# build GAMS
echo "Building GAMS"
cd $GAMS_ROOT
perl $ACE_ROOT/bin/mwc.pl -type make -features java=$JAVA,ros=$ROS,vrep=$VREP,tests=$TESTS,android=$ANDROID gams.mwc
make realclean -j $CORES
make java=$JAVA ros=$ROS vrep=$VREP tests=$TESTS android=$ANDROID -j $CORES 
ln -s $GAMS_ROOT/bin/gams_controller $GAMS_ROOT/gams_controller
ln -s $GAMS_ROOT/bin/dynamic_simulation $GAMS_ROOT/dynamic_simulation
if [ $JAVA -eq 1 ]; then
  # sometimes the jar'ing will occur before all classes are actually built when performing
  # multi-job builds, fix by deleting class files and recompiling with single build job
  find . -name "*.class" -delete
  make java=$JAVA ros=$ROS vrep=$VREP tests=$TESTS android=$ANDROID
fi
