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
# Copyright 2015 Carnegie Mellon University
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
# Build the required libraries for GAMS
#
# There are several expected environment variables
#   $CORES        - number of build jobs to launch with make
#   $MADARA_ROOT  - location of local copy of MADARA git repository from
#                   git://git.code.sf.net/p/madara/code
#   $GAMS_ROOT    - location of this GAMS git repository
#   $VREP_ROOT    - location of VREP installation, if applicable
#   $SSL_ROOT     - location of OpenSSL (usually /usr)
#   $ZMQ_ROOT     - location of ZeroMQ (usually /usr/local)
#   $LZ4_ROOT     - location of LZ4
#
# For android
#   $LOCAL_CROSS_PREFIX
#                 - Set this to the toolchain prefix
#
# For java
#   $JAVA_HOME

#COLORS
ORANGE='\033[0;33m'
BLUE='\033[0;34m'
NOCOLOR='\033[0m' 

CAPNP=0
# Hard setting this because of SCRIMMAGE debian bug
SCRIMMAGE_ROOT="/opt/scrimmage/x86_64-linux-gnu/" 

DEBUG=0
TESTS=0
TUTORIALS=0
VREP=0
SCRIMMAGE=0
JAVA=0
ROS=0
CLANG=0
ANDROID=0
STRIP=0
ODROID=0
MPC=0
MADARA=0
GAMS=0
PREREQS=0
DOCS=0
VREP_CONFIG=0
ZMQ=0
SIMTIME=0
NOTHREADLOCAL=0
SSL=0
DMPL=0
LZ4=0
NOKARL=0
PYTHON=0
WARNINGS=0
CLEAN=1
CLEAN_ENV=0
MADARAPULL=1
GAMSPULL=1
MAC=${MAC:-0}
BUILD_ERRORS=0
TYPES=0
ANDROID_TESTS=0
CAPNP_JAVA=0
UNREAL=0
AIRLIB=0
FORCE_UNREAL=0
FORCE_AIRSIM=0
UNREAL_DEV=0
UNREAL_GAMS=0
CLANG_DEFINED=0
CLANG_IN_LAST=0


MPC_DEPENDENCY_ENABLED=0
MADARA_DEPENDENCY_ENABLED=0
MPC_AS_A_PREREQ=0
MADARA_AS_A_PREREQ=0
VREP_AS_A_PREREQ=0
SCRIMMAGE_AS_A_PREREQ=0
GAMS_AS_A_PREREQ=0
EIGEN_AS_A_PREREQ=0
CAPNP_AS_A_PREREQ=0
UNREAL_AS_A_PREREQ=0

MPC_REPO_RESULT=0
DART_REPO_RESULT=0
DART_BUILD_RESULT=0
GAMS_REPO_RESULT=0
GAMS_BUILD_RESULT=0
MADARA_REPO_RESULT=0
MADARA_BUILD_RESULT=0
MPC_REPO_RESULT=0
VREP_REPO_RESULT=0
SCRIMMAGE_REPO_RESULT=0
ZMQ_REPO_RESULT=0
ZMQ_BUILD_RESULT=0
LZ4_REPO_RESULT=0
CAPNP_REPO_RESULT=0
CAPNP_BUILD_RESULT=0
CAPNPJAVA_REPO_RESULT=0
CAPNPJAVA_BUILD_RESULT=0
UNREAL_BUILD_RESULT=0
AIRSIM_BUILD_RESULT=0
UNREAL_GAMS_REPO_RESULT=0
UNREAL_GAMS_BUILD_RESULT=0

STRIP_EXE=strip
VREP_INSTALLER="V-REP_PRO_EDU_V3_4_0_Linux.tar.gz"
export INSTALL_DIR=`pwd`
SCRIPTS_DIR=`dirname $0`

if [ "$EUID" == 0 ] ; then
    echo "Please re-run this script without sudo."
    exit
fi

if [ -z $CORES ] ; then
  echo "CORES unset, so setting it to default of 1"
  echo "  If you have more than one CPU core, try export CORES=<num cores>"
  echo "  CORES=1 (the default) will be much slower than CORES=<num cores>"
  export CORES=1
fi

# if $@ is empty, the user wants to repeat last build with noclean

if [ $# == 0 ]; then
  echo "Loading last build with noclean..."
  IFS=$'\r\n ' GLOBIGNORE='*' command eval  'ARGS=($(cat $GAMS_ROOT/last_build.lst))'
  ARGS+=("noclean")
else
  echo "Processing user arguments..."
  ARGS=("$@")
fi

echo "build features: ${ARGS[@]}"

for var in "${ARGS[@]}"
do
  if [ "$var" = "airlib" ] ||  [ "$var" = "airsim" ]; then
    AIRLIB=1
    CLANG=1
  elif [ "$var" = "android" ]; then
    ANDROID=1
    JAVA=1
    STRIP_EXE=${LOCAL_CROSS_PREFIX}strip
  elif [ "$var" = "android-tests" ]; then 
    ANDROID_TESTS=1
    ANDROID=1
    JAVA=1
  elif [ "$var" = "capnp" ]; then
    CAPNP=1
  elif [ "$var" = "nocapnp" ]; then
    CAPNP=0
  elif [ "$var" = "capnp-java" ]; then
    CAPNP_JAVA=1
  elif [ "$var" = "clang" ]; then
    CLANG=1
    CLANG_DEFINED=1
  elif [ "$var" = "clang5" ] ||  [ "$var" = "clang-5" ]; then
    CLANG=1
    CLANG_DEFINED=1
    export CLANG_SUFFIX=-5.0
    export FORCE_CC=clang-5.0
    export FORCE_CXX=clang++-5.0
  elif [ "$var" = "clang6" ] ||  [ "$var" = "clang-6" ]; then
    CLANG=1
    CLANG_DEFINED=1
    export CLANG_SUFFIX=-6.0
    export FORCE_CC=clang-6.0
    export FORCE_CXX=clang++-6.0
  elif [ "$var" = "clang8" ] ||  [ "$var" = "clang-8" ]; then
    CLANG=1
    CLANG_DEFINED=1
    export CLANG_SUFFIX=-8
    export FORCE_CC=clang-8
    export FORCE_CXX=clang++-8
  elif [ "$var" = "clang9" ] ||  [ "$var" = "clang-9" ]; then
    CLANG=1
    CLANG_DEFINED=1
    export CLANG_SUFFIX=-9
    export FORCE_CC=clang-9
    export FORCE_CXX=clang++-9
  elif [ "$var" = "clean" ]; then
    CLEAN=1
  elif [ "$var" = "cleanenv" ]; then
    CLEAN_ENV=1
  elif [ "$var" = "dart" ]; then
    DMPL=1
  elif [ "$var" = "debug" ]; then
    DEBUG=1
  elif [ "$var" = "dmpl" ]; then
    DMPL=1
  elif [ "$var" = "docs" ]; then
    DOCS=1
  elif [ "$var" = "force-airsim" ]; then
    AIRLIB=1
    FORCE_AIRSIM=1
  elif [ "$var" = "force-unreal" ]; then
    UNREAL=1
    FORCE_UNREAL=1
  elif [ "$var" = "gams" ]; then
    GAMS=1
  elif [ "$var" = "java" ]; then
    JAVA=1
  elif [ "$var" = "lz4" ]; then
    LZ4=1
  elif [ "$var" = "mpc" ]; then
    MPC=1
  elif [ "$var" = "madara" ]; then
    MADARA=1
  elif [ "$var" = "noclean" ]; then
    CLEAN=0
  elif [ "$var" = "nogamspull" ]; then
    GAMSPULL=0
  elif [ "$var" = "nokarl" ]; then
    NOKARL=1
  elif [ "$var" = "nomadarapull" ]; then
    MADARAPULL=0
  elif [ "$var" = "nopull" ]; then
    GAMSPULL=0
    MADARAPULL=0
  elif [ "$var" = "nothreadlocal" ]; then
    NOTHREADLOCAL=1
  elif [ "$var" = "odroid" ]; then
    ODROID=1
    STRIP_EXE=${LOCAL_CROSS_PREFIX}strip
  elif [ "$var" = "prereqs" ]; then
    PREREQS=1
  elif [ "$var" = "python" ]; then
    PYTHON=1
  elif [ "$var" = "ros" ]; then
    ROS=1
  elif [ "$var" = "simtime" ]; then
    SIMTIME=1
  elif [ "$var" = "ssl" ]; then
    SSL=1
  elif [ "$var" = "strip" ]; then
    STRIP=1
  elif [ "$var" = "tests" ]; then
    TESTS=1
  elif [ "$var" = "tutorials" ]; then
    TUTORIALS=1
  elif [ "$var" = "types" ]; then
    TYPES=1
  elif [ "$var" = "unreal" ]; then
    UNREAL=1
  elif [ "$var" = "unreal-dev" ]; then
    UNREAL_DEV=1
    UNREAL_GAMS=1
    CLANG=1
  elif [ "$var" = "unreal-gams" ]; then
    UNREAL_GAMS=1
  elif [ "$var" = "scrimmage" ]; then
    SCRIMMAGE=1
  elif [ "$var" = "vrep" ]; then
    VREP=1
  elif [ "$var" = "vrep-config" ]; then
    VREP_CONFIG=1
  elif [ "$var" = "warnings" ]; then
    WARNINGS=1
  elif [ "$var" = "zmq" ]; then
    ZMQ=1
  else
#    echo "Invalid argument: $var"
    echo ""
    echo "Args can be zero or more of the following, space delimited"
    echo ""
    echo "  airlib|airsim   build with Microsoft AirSim support"
    echo "  android         build android libs, turns on java"
    echo "  capnp           enable capnproto support"
    echo "  clang           build using clang++\$CLANG_SUFFIX and libc++"
    echo "  clang-5         build using clang++-5.0 and libc++"
    echo "  clang-6         build using clang++-6.0 and libc++"
    echo "  clang-8         build using clang++-8.0 and libc++"
    echo "  clang-9         build using clang++-9.0 and libc++"
    echo "  clean           run 'make clean' before builds (default)"
    echo "  cleanenv       Unsets all related environment variables before building."
    echo "  debug           create a debug build, with minimal optimizations"
    echo "  dmpl            build DART DMPL verifying compiler"
    echo "  docs            generate API documentation"
    echo "  force-airsim    if airsim dir exists, rebuild"
    echo "  force-unreal    if unreal dir exists, rebuild"
    echo "  gams            build GAMS"
    echo "  help            get script usage"
    echo "  java            build java jar"
    echo "  lz4             build with LZ4 compression"
    echo "  madara          build MADARA"
    echo "  mpc             download MPC if prereqs is enabled"
    echo "  nocapnp         disable capnproto support"
    echo "  noclean         do not run 'make clean' before builds."
    echo "                  This is an option that supercharges the build"
    echo "                  process and can reduce build times to seconds."
    echo "                  99.9% of update builds can use this, unless you"
    echo "                  are changing features (e.g., enabling ssl when"
    echo "                  you had previously not enabled ssl)"
    echo "  nogamspull      when building GAMS, don't do a git pull"
    echo "  nokarl          when building MADARA, remove all karl evaluation"
    echo "                  This is useful to remove RTTI dependencies"
    echo "  nomadarapull    when building MADARA, don't do a git pull"
    echo "  nopull          when building MADARA or GAMS, don't do a git pull"
    echo "  odroid          target ODROID computing platform"
    echo "  python          build with Python 2.7 support"
    echo "  prereqs         use apt-get to install prereqs. This usually only"
    echo "                  has to be used on the first usage of a feature"
    echo "  ros             build ROS platform classes"
    echo "  ssl             build with OpenSSL support"
    echo "  simtime         build with simtime support in Madara"
    echo "  strip           strip symbols from the libraries"
    echo "  tests           build test executables"
    echo "  tutorials       build MADARA tutorials"
    echo "  types           builds libTYPES.so"
    echo "  unreal          builds Unreal and AirSim if preqreqs is enabled"
    echo "  unreal-dev      builds Unreal Dev and UnrealGAMS"
    echo "  unreal-gams     builds UnrealGAMS"
    echo "  vrep            build with vrep support"
    echo "  scrimmage-gams  build with scrimmage support"
    echo "  vrep-config     configure vrep to support up to 20 agents"
    echo "  warnings        build with compile warnings enabled in GAMS/MADARA"
    echo "  zmq             build with ZeroMQ support"
    echo ""
    echo "The following environment variables are used"
    echo ""
    echo "  AIRSIM_ROOT         - location of AirSim repository"
    echo "  CAPNP_ROOT          - location of Cap'n Proto"
    echo "  CORES               - number of build jobs to launch with make, optional"
    echo "  DMPL_ROOT           - location of DART DMPL directory"
    echo "  MPC_ROOT            - location of MakefileProjectCreator"
    echo "  MADARA_ROOT         - location of local copy of MADARA git repository from"
    echo "                        git://git.code.sf.net/p/madara/code"
    echo "  GAMS_ROOT           - location of this GAMS git repository"
    echo "  VREP_ROOT           - location of VREP installation"
    echo "  SCRIMMAGE_GIT_ROOT  - the location of the SCRIMMAGE Github installation"
    echo "  SCRIMMAGE_ROOT      - the location of the SCRIMMAGE installation"
    echo "  JAVA_HOME           - location of JDK"
    echo "  LZ4_ROOT            - location of LZ4"
    echo "  MPC_ROOT            - location of MakefileProjectCreator"
    echo "  MADARA_ROOT         - location of local copy of MADARA repository"
    echo "  ROS_ROOT            - location of ROS (usually set by ROS installer)"
    echo "  SSL_ROOT            - location of OpenSSL"
    echo "  UNREAL_ROOT         - location of UnrealEngine repository"
    echo "  VREP_ROOT           - location of VREP installation"
    echo "  ZMQ_ROOT            - location of ZeroMQ"
    echo ""
    echo "Previous build (can repeat by calling this script with no args):"
    echo ""
    echo "  $(cat $GAMS_ROOT/last_build.lst)"
    echo ""
    exit
  fi
done

# make the .gams directory if it doesn't exist
if [ ! -d $HOME/.gams ]; then
  mkdir $HOME/.gams
  touch $HOME/.gams/env.sh
elif [ $CLEAN_ENV -eq 1 ]; then
  rm $HOME/.gams/env.sh
  touch $HOME/.gams/env.sh
fi

if [ $CLANG -eq 1 ] && [ $CLANG_DEFINED -eq 0 ] ; then
  ARGS+=('clang')
fi

# did we compile with clang last time?
if grep -q clang $GAMS_ROOT/last_build.lst ; then
  CLANG_IN_LAST=1
fi

# if we have changed compilers, stop everything
if [ $CLANG_IN_LAST -ne $CLANG ] ; then
  echo "Compiler change detected. Forcing clean build"

  for i in "${!ARGS[@]}"; do
    if [ "${ARGS[i]}" != "noclean" ]; then
      NEW_ARGS+=( "${ARGS[i]}" )
    fi
  done
  ARGS=("${NEW_ARGS[@]}")
  unset NEW_ARGS
  CLEAN=1

  if [ $MADARA -eq 0 ] && [ $GAMS -eq 1 ] ; then
    echo "  MADARA needs to be rebuilt"
    MADARA_AS_A_PREREQ=1
  fi
fi

if [ $DMPL -eq 1 ] || [ $GAMS -eq 1 ] ; then
  MADARA_DEPENDENCY_ENABLED=1
fi

# check if MADARA is a prereq for later packages
if [ $MADARA_DEPENDENCY_ENABLED -eq 1 ] && [ ! -d $MADARA_ROOT ]; then
  MADARA_AS_A_PREREQ=1
fi

# save the last builds that have been performed
echo "${ARGS[*]}" >> $HOME/.gams/build.lst

CLANG_IN_LAST=0

if [ $CORES -eq 1 ] ; then
    echo -e "${ORANGE} Warning! The CORES environment variable is set to 1. With only one core installation will take longer than it could be. By increasing the number of cores to compile with you will be reducing compilation time substantially. ${NOCOLOR}"
    CORE_COUNT=$(grep -c ^processor /proc/cpuinfo)
    echo "The system has ${CORE_COUNT} cores. How many should be used for installation?"
    read CORES
    echo -e "${ORANGE} Now using $CORES CPU cores for installation. To permanently set this, add the line 'export CORES=$CORES to your ~/.bashrc file and open a new terminal. ${NOCOLOR}"
fi

if [ $CLEAN_ENV -eq 1 ]; then
    echo -e "${ORANGE} Resetting all environment variables to blank. ${NOCOLOR}"
    export UNREAL_ROOT=""
    export UNREAL_GAMS_ROOT=""
    export UE4_ROOT=""
    export UE4_GAMS=""
    export AIRSIM_ROOT=""
    export OSC_ROOT=""
    export VREP_ROOT=""
    export MPC_ROOT=""
    export EIGEN_ROOT=""
    export CAPNP_ROOT=""
    export MADARA_ROOT=""
    export GAMS_ROOT=""
    export DMPL_ROOT=""
    export SCRIMMAGE_PLUGIN_PATH=$SCRIMMAGE_PLUGIN_PATH$GAMS_ROOT/lib/scrimmage_plugins
    export PYTHONPATH=$PYTHONPATH:$MADARA_ROOT/lib:$GAMS_ROOT/lib
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MADARA_ROOT/lib:$GAMS_ROOT/lib:$VREP_ROOT:$CAPNP_ROOT/c++/.libs
    export PATH=$PATH:$MPC_ROOT:$VREP_ROOT:$CAPNP_ROOT/c++:$MADARA_ROOT/bin:$GAMS_ROOT/bin:$DMPL_ROOT/src/DMPL:$DMPL_ROOT/src/vrep
    export SCRIMMAGE_GIT_ROOT=""
fi

if [ -z $DMPL_ROOT ] ; then
  export DMPL_ROOT=$INSTALL_DIR/dmplc
fi

if [ -z $MPC_ROOT ] ; then
  export MPC_ROOT=$INSTALL_DIR/MPC
fi

if [ -z $EIGEN_ROOT ] ; then
  export EIGEN_ROOT=$INSTALL_DIR/eigen
fi

if [ -z $CAPNP_ROOT ] ; then
  export CAPNP_ROOT=$INSTALL_DIR/capnproto
fi

if [ -z $CAPNPJAVA_ROOT ] ; then
  export CAPNPJAVA_ROOT=$INSTALL_DIR/capnproto-java
fi

if [ -z $LZ4_ROOT ] ; then
  export LZ4_ROOT=$INSTALL_DIR/lz4
fi

if [ -z $OSC_ROOT ] ; then
  export OSC_ROOT=$INSTALL_DIR/oscpack
fi

if [ -z $VREP_ROOT ] ; then
  export VREP_ROOT=$INSTALL_DIR/vrep
fi

if [ -z $SCRIMMAGE_GIT_ROOT ] ; then
  export SCRIMMAGE_GIT_ROOT="$INSTALL_DIR/scrimmage"
fi

if [ -z $UNREAL_ROOT ] ; then
  export UNREAL_ROOT=$INSTALL_DIR/UnrealEngine
  export UE4_ROOT=$INSTALL_DIR/UnrealEngine
fi

if [ -z $AIRSIM_ROOT ] ; then
  export AIRSIM_ROOT=$INSTALL_DIR/AirSim
fi

if [ -z $UNREAL_GAMS_ROOT ] ; then
  export UNREAL_GAMS_ROOT=$INSTALL_DIR/UnrealGAMS
  export UE4_GAMS=$INSTALL_DIR/UnrealGAMS
fi

if [ ! -z $CC ] ; then
  export FORCE_CC=$CC
  echo "Forcing CC=$CC"
fi

if [ ! -z $CXX ] ; then
  export FORCE_CXX=$CXX
  echo "Forcing CXX=$CXX"
fi

# echo build information
echo "INSTALL_DIR will be $INSTALL_DIR"
echo "Using $CORES build jobs"

if [ $PREREQS -eq 0 ]; then
  echo "No pre-requisites will be installed"
else
  echo "Pre-requisites will be installed"
fi

echo "MPC_ROOT is set to $MPC_ROOT"

echo "EIGEN_ROOT is set to $EIGEN_ROOT"
echo "CAPNP_ROOT is set to $CAPNP_ROOT"
echo "CAPNPJAVA_ROOT is set to $CAPNPJAVA_ROOT"
echo "OSC_ROOT is set to $OSC_ROOT"
echo "UNREAL_ROOT is set to $UNREAL_ROOT"
echo "AIRSIM_ROOT is set to $AIRSIM_ROOT"
echo "LZ4_ROOT is set to $LZ4_ROOT"
echo "MADARA will be built from $MADARA_ROOT"
if [ $MADARA -eq 0 ]; then
  echo "MADARA will not be built"
else
  echo "MADARA will be built"
fi

echo "GAMS_ROOT is set to $GAMS_ROOT"

echo "ODROID has been set to $ODROID"
echo "TESTS has been set to $TESTS"
echo "ROS has been set to $ROS"
echo "STRIP has been set to $STRIP"
if [ $STRIP -eq 1 ]; then
  echo "strip will use $STRIP_EXE"
fi
echo "AIRLIB has been set to $AIRLIB"

echo "JAVA has been set to $JAVA"
if [ $JAVA -eq 1 ]; then
  echo "JAVA_HOME is referencing $JAVA_HOME"
fi

echo "VREP has been set to $VREP"
if [ $VREP -eq 1 ]; then
  echo "VREP_ROOT is referencing $VREP_ROOT"
fi

echo "SCRIMMAGE has been set to $SCRIMMAGE"
if [ $SCRIMMAGE -eq 1 ]; then
  echo "SCRIMMAGE_GIT_ROOT is referencing $SCRIMMAGE_GIT_ROOT"
fi

echo "ANDROID has been set to $ANDROID"
if [ $ANDROID -eq 1 ]; then
  echo "CROSS_COMPILE is set to $LOCAL_CROSS_PREFIX"
  if [ -z "$ANDROID_API" ]; then
    export ANDROID_API=27
    echo "ANDROID_API is unset; defaulting to $ANDROID_API"
  else
    echo "ANDROID_API has been set to $ANDROID_API"
  fi
  if [ -z "$ANDROID_ABI" ]; then
    export ANDROID_ABI=4.9
    echo "ANDROID_ABI is unset; defaulting to $ANDROID_ABI"
  else
    echo "ANDROID_ABI has been set to $ANDROID_ABI"
  fi
  if [ -z "$ANDROID_ARCH" ]; then
    export ANDROID_ARCH=armeabi-v7a
    echo "ANDROID_ARCH is unset; defaulting to $ANDROID_ARCH"
  else
    echo "ANDROID_ARCH has been set to $ANDROID_ARCH"
  fi
  if [ -z "$NDK_VERSION" ]; then
    export NDK_VERSION=r17c
    echo "NDK_VERSION is unset; defaulting to $NDK_VERSION"
  else
    echo "NDK_VERSION has been set to $NDK_VERSION"
  fi
  if [ -z "$NDK_ROOT" ]; then
    export NDK_ROOT=$INSTALL_DIR/ndk
    echo "NDK_ROOT is unset; defaulting to $NDK_ROOT"
  else
    echo "NDK_ROOT has been set to $NDK_ROOT"
  fi
  if [ -z "$NDK_TOOLS" ]; then
    export NDK_TOOLS=$NDK_ROOT/custom_toolchain
    echo "NDK_TOOLS is unset; defaulting to $NDK_TOOLS"
  else
    echo "NDK_TOOLS has been set to $NDK_TOOLS"
  fi
  if [ -z "$SYSROOT" ]; then
    export SYSROOT=$NDK_TOOLS/sysroot
    echo "SYSROOT is unset; defaulting to $SYSROOT"
  else
    echo "SYSROOT has been set to $SYSROOT"
  fi
  
  case $ANDROID_ARCH in
    arm32|arm|armeabi|armeabi-v7a)
      export ANDROID_ARCH=armeabi-v7a;
      export ANDROID_TOOLCHAIN_ARCH=arm;
      export ANDROID_TOOLCHAIN=arm-linux-androideabi;;

    arm64|aarch64)
      export ANDROID_ARCH=aarch64;
      export ANDROID_TOOLCHAIN_ARCH=arm64;
      export ANDROID_TOOLCHAIN=aarch64-linux-android;;

    x86)
      true;;

    x64|x86_64)
      export ANDROID_ARCH=x86_64;;

    *)
      echo "Unknown ANDROID_ABI: $ANDROID_ABI"; exit 1 ;;
  esac
  export ANDROID_TOOLCHAIN=${ANDROID_TOOLCHAIN:-${ANDROID_TOOLCHAIN_ARCH}-$ANDROID_ABI}
  export ANDROID_TOOLCHAIN_ARCH=${ANDROID_TOOLCHAIN_ARCH:-$ANDROID_ARCH}
  echo "ANDROID_TOOLCHAIN is set to $ANDROID_TOOLCHAIN"

  export PATH="$NDK_TOOLS/bin:$PATH"


  if [ -z "$BOOST_ANDROID_ROOT" ]; then
    export BOOST_ANDROID_ROOT=$INSTALL_DIR/BoostAndroid
    echo "Boost root is set to $BOOST_ANDROID_ROOT is not set."
  fi

  if [ $SSL -eq 1 ] && [ -z $SSL_ROOT ]; then
      export SSL_ROOT=$INSTALL_DIR/openssl;
  fi

fi

if [ $DOCS -eq 1 ]; then
  echo "DOCS is set to $DOCS"
fi

echo ""

unzip_strip() (
  local zip=$1
  local dest=${2:-.}
  local temp=$(mktemp -d) && unzip -qq -d "$temp" "$zip" && mkdir -p "$dest" &&
  shopt -s dotglob && local f=("$temp"/*) &&
  if (( ${#f[@]} == 1 )) && [[ -d "${f[0]}" ]] ; then
    mv "$temp"/*/* "$dest"
  else
    mv "$temp"/* "$dest"
  fi && rmdir "$temp"/* "$temp"
)

append_if_needed() (
  if grep -q "$1" "$2"; then
    true
  else
    echo "$1" >> "$2"
  fi
)

if [ $PREREQS -eq 1 ] && [ $MAC -eq 0 ]; then
  if [ $JAVA -eq 1 ]; then
    sudo add-apt-repository -y ppa:webupd8team/java
  fi

  if [ $ROS -eq 1 ]; then
    ROS_FIRST_SETUP=0
    if [ ! -d "/opt/ros/kinetic" ] ; then
      ROS_FIRST_SETUP=1
      sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
      sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    fi
  fi

  sudo apt-get update
  sudo apt-get install -y -f build-essential subversion git-core perl doxygen graphviz libboost-all-dev
  sudo apt-get install -y -f autoconf automake libtool

  if [ $CLANG -eq 1 ]; then
    sudo apt-get install -y -f clang-6.0 libc++-dev libc++abi-dev clang-5.0

    if [ $CLANG_SUFFIX = "-8" ]; then
      sudo apt-get install -y -f clang-8
    elif [ $CLANG_SUFFIX = "-9" ]; then
      sudo apt-get install -y -f clang-9
    fi
  fi

  if [ $JAVA -eq 1 ] && [ -z $JAVA_HOME ]; then
    sudo apt-get install -y -f oracle-java8-set-default
    sudo apt-get install -y maven
    export JAVA_HOME=/usr/lib/jvm/java-8-oracle
    rc_str="export JAVA_HOME=$JAVA_HOME"
    append_if_needed "$rc_str" "$HOME/.bashrc"
  
  fi

  if [ $PYTHON -eq 1 ]; then
    sudo apt-get install -y -f python2.7 python-pip python-tk
    sudo pip install matplotlib
    sudo pip install pycapnp
    sudo pip install pyyaml
    sudo pip install yamlloader
  fi

  if [ $ANDROID -eq 1 ]; then
    sudo apt-get install -y -f gcc-arm-linux-androideabi

    if [ ! -f $NDK_ROOT/README.md ]; then
      mkdir -p $NDK_ROOT;
      (
        cd $NDK_ROOT;
        NDK_ZIP="android-ndk-${NDK_VERSION}-linux-x86_64.zip"
        if [ ! -f $NDK_ZIP ]; then
          wget "https://dl.google.com/android/repository/$NDK_ZIP" || exit $?
        fi
        if [ ! -f $NDK_ZIP ]; then
          echo Failed to download $NDK_ZIP
          exit 1
        fi
        unzip_strip $NDK_ZIP . || exit $?
      ) || exit $?
    fi
    (
      cd $NDK_ROOT;
      ./build/tools/make_standalone_toolchain.py --force \
            --api=$ANDROID_API \
            --install-dir=$NDK_TOOLS --stl=libc++ --arch=$ANDROID_TOOLCHAIN_ARCH || exit $?
    ) || exit $?

     if [ ! -d $BOOST_ANDROID_ROOT ] || [ ! -d "$BOOST_ANDROID_ROOT/build/$ANDROID_ARCH" ]; then
       git clone "https://github.com/amsurana/Boost-for-Android.git" $BOOST_ANDROID_ROOT
       cd $BOOST_ANDROID_ROOT;
       echo "Boost is cloned in $BOOST_ANDROID_ROOT"
       ./build-android.sh --boost=1.65.1 --arch=$ANDROID_ARCH $NDK_ROOT
     fi
     
     if [ ! -d $BOOST_ANDROID_ROOT ] || [ ! -d "$BOOST_ANDROID_ROOT/build/$ANDROID_ARCH" ]; then
       echo "Unable to download or setup boos. Refer README-ANDROID.md for manual setup"
       exit 1;
     fi 

     if [ $SSL -eq 1 ]; then
        echo "SSL_ROOT is set to $SSL_ROOT"
        if [ ! -d $SSL_ROOT ]; then
          git clone --depth 1 https://github.com/openssl/openssl.git $SSL_ROOT
        fi
        cd $SSL_ROOT
        export ANDROID_NDK=$NDK_ROOT
        HOST_ARCH="linux-x86_64";
        if [ $MAC == 1 ]; then
          HOST_ARCH="darwin-x86_64";
        fi 
        export PATH=$ANDROID_NDK/toolchains/$ANDROID_TOOLCHAIN-$ANDROID_ABI/prebuilt/$HOST_ARCH/bin:$PATH
        echo $PATH;
        ./Configure android-arm
        make clean
        make -j4
     fi

  fi #android condition ends

  if [ $ROS -eq 1 ]; then
    sudo apt-get install -y ros-kinetic-desktop-full python-rosinstall ros-kinetic-ros-type-introspection ros-kinetic-move-base-msgs ros-kinetic-navigation libactionlib-dev libactionlib-msgs-dev libmove-base-msgs-dev ros-kinetic-pcl-conversions ros-kinetic-pcl-ros libpcl-dev

    if [ $ROS_FIRST_SETUP -eq 1 ]; then
      sudo rosdep init
      rosdep update
      echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
      source ~/.bashrc
    fi
  fi

  if [ $SSL -eq 1 ]; then
    sudo apt-get install -y libssl-dev
  fi
  
  if [ $ZMQ -eq 1 ]; then 
    sudo apt-get install -y libtool pkg-config autoconf automake
  fi

  if [ $DMPL -eq 1 ]; then 
    sudo apt-get install -y perl git build-essential subversion libboost-all-dev bison flex realpath cbmc tk xvfb libyaml-cpp-dev ant
  fi

  if [ $UNREAL -eq 1 ] || [ $UNREAL_DEV -eq 1 ] ; then

    if [ ! -d "$UNREAL_ROOT" ]; then
      echo "Installing UnrealEngine"
      echo "git clone -b 4.21 git@github.com:EpicGames/UnrealEngine.git $UNREAL_ROOT"
      git clone -b 4.21 git@github.com:EpicGames/UnrealEngine.git $UNREAL_ROOT
      FORCE_UNREAL=1 
    fi

    if [ $FORCE_UNREAL -eq 1 ] ; then
      cd $UNREAL_ROOT
      echo "./Setup.sh"
      ./Setup.sh
      echo "./GenerateProjectFiles.sh"
      ./GenerateProjectFiles.sh
      echo "Making unreal"
      make
      UNREAL_BUILD_RESULT=$?
    else
      echo -e "${BLUE}Skipping unreal re-installation... UnrealEngine already installed at $UNREAL_ROOT${NOCOLOR}"
    fi

    if [ ! -d "$AIRSIM_ROOT" ] && [ $AIRLIB -eq 1 ] ; then
      echo "git clone https://github.com/Microsoft/AirSim.git $AIRSIM_ROOT"
      git clone https://github.com/Microsoft/AirSim.git $AIRSIM_ROOT
      FORCE_AIRSIM=1
    fi

    if [ $AIRLIB -eq 1 ] && [ $FORCE_AIRSIM -eq 1 ] ; then
      echo "Building AirSim"
      cd $AIRSIM_ROOT
      ./setup.sh
      ./build.sh
      AIRSIM_BUILD_RESULT=$?
    else
      echo -e "${BLUE}Skipping AirSim re-installation... AirSim already installed at $AIRSIM_ROOT ${NOCOLOR}"
    fi
    
    echo "Installation complete. Open UnrealEngine/Engine/Binaries/Linux/UE4Editor and then When Unreal Engine prompts for opening or creating project, select Browse and choose AirSim/Unreal/Environments/Blocks. For more information see \" How to Use AirSim\" at https://microsoft.github.io/AirSim/docs/build_linux/"
  fi
fi
if [ $MAC -eq 1 ]; then
  # Install boost for mac
  if [ $PREREQS -eq 1 ]; then
    brew install boost@1.59
    brew install autoconf automake libtool
    if [ $JAVA -eq 1 ]; then
      brew install maven
    fi
  fi
  export BOOST_ROOT=/usr/local/opt/boost@1.59/include
  export BOOST_ROOT_LIB=/usr/local/opt/boost@1.59/lib
fi

# Update CORES in the GAMS environment file
if grep -q CORES $HOME/.gams/env.sh ; then
  sed -i 's@CORES=.*@CORES='"$CORES"'@' $HOME/.gams/env.sh
else
  echo "export CORES=$CORES" >> $HOME/.gams/env.sh
fi

# Update UNREAL_ROOT in the GAMS environment file
if grep -q UNREAL_ROOT $HOME/.gams/env.sh ; then
  sed -i 's@UNREAL_ROOT=.*@UNREAL_ROOT='"$UNREAL_ROOT"'@' $HOME/.gams/env.sh
else
  echo "export UNREAL_ROOT=$UNREAL_ROOT" >> $HOME/.gams/env.sh
fi

# Update UNREAL_ROOT in the GAMS environment file
if grep -q UNREAL_GAMS_ROOT $HOME/.gams/env.sh ; then
  sed -i 's@UNREAL_GAMS_ROOT=.*@UNREAL_GAMS_ROOT='"$UNREAL_GAMS_ROOT"'@' $HOME/.gams/env.sh
else
  echo "export UNREAL_GAMS_ROOT=$UNREAL_GAMS_ROOT" >> $HOME/.gams/env.sh
fi

# Update UE4_ROOT in the GAMS environment file
if grep -q UE4_ROOT $HOME/.gams/env.sh ; then
  sed -i 's@UE4_ROOT=.*@UE4_ROOT='"$UE4_ROOT"'@' $HOME/.gams/env.sh
else
  echo "export UE4_ROOT=$UE4_ROOT" >> $HOME/.gams/env.sh
fi

# Update UE4_GAMS in the GAMS environment file
if grep -q UE4_GAMS $HOME/.gams/env.sh ; then
  sed -i 's@UE4_GAMS=.*@UE4_GAMS='"$UE4_GAMS"'@' $HOME/.gams/env.sh
else
  echo "export UE4_GAMS=$UE4_GAMS" >> $HOME/.gams/env.sh
fi

# Update AIRSIM_ROOT in the GAMS environment file
if grep -q AIRSIM_ROOT $HOME/.gams/env.sh ; then
  sed -i 's@AIRSIM_ROOT=.*@AIRSIM_ROOT='"$AIRSIM_ROOT"'@' $HOME/.gams/env.sh
else
  echo "export AIRSIM_ROOT=$AIRSIM_ROOT" >> $HOME/.gams/env.sh
fi

# Update OSC_ROOT in the GAMS environment file
if grep -q OSC_ROOT $HOME/.gams/env.sh ; then
  sed -i 's@OSC_ROOT=.*@OSC_ROOT='"$OSC_ROOT"'@' $HOME/.gams/env.sh
else
  echo "export OSC_ROOT=$OSC_ROOT" >> $HOME/.gams/env.sh
fi

# Update GAMS environment script with VREP_ROOT
if grep -q VREP_ROOT $HOME/.gams/env.sh ; then
  sed -i 's@VREP_ROOT=.*@VREP_ROOT='"$VREP_ROOT"'@' $HOME/.gams/env.sh
else
  echo "export VREP_ROOT=$VREP_ROOT" >> $HOME/.gams/env.sh
fi

# Update GAMS environment script with SCRIMMAGE_GIT_ROOT
if grep -q SCRIMMAGE_GIT_ROOT $HOME/.gams/env.sh ; then
  sed -i 's@SCRIMMAGE_GIT_ROOT=.*@SCRIMMAGE_GIT_ROOT='"$SCRIMMAGE_GIT_ROOT"'@' $HOME/.gams/env.sh
else
  echo "export SCRIMMAGE_GIT_ROOT=$SCRIMMAGE_GIT_ROOT" >> $HOME/.gams/env.sh
fi

# Update GAMS environment script with SCRIMMAGE_ROOT
# Only have to do this because SCRIMMAGE doesnt' set it in their own files.

# Update GAMS environment script with SCRIMMAGE_ROOT
if grep -q SCRIMMAGE_ROOT $HOME/.gams/env.sh ; then
  sed -i 's@SCRIMMAGE_ROOT=.*@SCRIMMAGE_ROOT='"$SCRIMMAGE_ROOT"'@' $HOME/.gams/env.sh
else
  echo "export SCRIMMAGE_GIT_ROOT=$SCRIMMAGE_GIT_ROOT" >> $HOME/.gams/env.sh
fi

if [ $LZ4 -eq 1 ] ; then
  export LZ4=1
  if [ ! -d $LZ4_ROOT  ]; then
    echo "git clone https://github.com/lz4/lz4.git $LZ4_ROOT"
    git clone https://github.com/lz4/lz4.git $LZ4_ROOT
  else
    echo "UPDATING LZ4"
    cd $LZ4_ROOT
    git pull
  fi

  if [ $ANDROID -eq 1 ]; then
    cd $LZ4_ROOT
    export CROSS_PATH=${NDK_TOOLS}/bin/${ANDROID_TOOLCHAIN}
    export AR=${CROSS_PATH}-ar
    export AS=${CROSS_PATH}-as
    export NM=${CROSS_PATH}-nm
    export CC=${CROSS_PATH}-gcc
    export CXX=${CROSS_PATH}-clang++
    export LD=${CROSS_PATH}-ld
    export RANLIB=${CROSS_PATH}-ranlib
    export STRIP=${CROSS_PATH}-strip

    make clean
    make 

  fi

  LZ4_REPO_RESULT=$?

  # Update GAMS environment script with LZ4_ROOT
  if grep -q LZ4_ROOT $HOME/.gams/env.sh ; then
    sed -i 's@LZ4_ROOT=.*@LZ4_ROOT='"$LZ4_ROOT"'@' $HOME/.gams/env.sh
  else
    echo "export LZ4_ROOT=$LZ4_ROOT" >> $HOME/.gams/env.sh
  fi

  if [ -f $LZ4_ROOT/lib/lz4.c ] ; then
    echo "cp $LZ4_ROOT/lib/lz4.c $LZ4_ROOT/lib/lz4.cpp"
    cp $LZ4_ROOT/lib/lz4.c $LZ4_ROOT/lib/lz4.cpp
  fi

  if [ ! -f $LZ4_ROOT/lib/liblz4.so ]; then
    echo "LZ4 library did not build properly";
  fi
fi

if [ ! -z $FORCE_CC ] ; then
  export CC=$FORCE_CC
  echo "Forcing CC=$CC"
fi

if [ ! -z $FORCE_CXX ] ; then
  export CXX=$FORCE_CXX
  echo "Forcing CXX=$CXX"
fi

# check if MPC is a prereq for later packages

if [ $DMPL -eq 1 ] || [ $GAMS -eq 1 ] || [ $MADARA -eq 1 ]; then
  MPC_DEPENDENCY_ENABLED=1
fi

if [ $MPC_DEPENDENCY_ENABLED -eq 1 ] && [ ! -d $MPC_ROOT ]; then
  MPC_AS_A_PREREQ=1
fi

#none of these appeared to work for me in 18.04 /bin/bash
#if { [ $MADARA -eq 1 ] || [ $MADARA_AS_A_PREREQ -eq 1 ]; } && [ $PREREQS -eq 1 ]; then
#if [[ ($MADARA -eq 1 ] || [ $MADARA_AS_A_PREREQ -eq 1) && $PREREQS -eq 1 ]]; then
if [ $PREREQS -eq 1 ] && [ $CAPNP -eq 1 ]; then
  if [ $MADARA -eq 1 ] || [ $MADARA_AS_A_PREREQ -eq 1 ]; then
    CAPNP_AS_A_PREREQ=1
  fi
fi

if [ $MPC -eq 1 ] || [ $MPC_AS_A_PREREQ -eq 1 ]; then

  cd $INSTALL_DIR

  echo "ENTERING $MPC_ROOT"
  if [ ! -d $MPC_ROOT ] ; then
    echo "git clone --depth 1 https://github.com/DOCGroup/MPC.git $MPC_ROOT"
    git clone --depth 1 https://github.com/DOCGroup/MPC.git $MPC_ROOT
    MPC_REPO_RESULT=$?
  fi
else
  echo "NOT CHECKING MPC"
fi

if [ $UNREAL_GAMS -eq 1 ] ; then

  cd $INSTALL_DIR

  if [ ! -d $UNREAL_GAMS_ROOT ] ; then
    echo "CLONING UnrealGAMS"
    echo "git clone git@github.com:VertexStudio/UnrealGAMS.git $UNREAL_GAMS_ROOT"
    git clone git@github.com:VertexStudio/UnrealGAMS.git $UNREAL_GAMS_ROOT
    UNREAL_GAMS_REPO_RESULT=$?
  else
    echo "UPDATING UnrealGAMS"
    cd $UNREAL_GAMS_ROOT
    git pull
    UNREAL_GAMS_REPO_RESULT=$?
  fi

  cd $UNREAL_GAMS_ROOT

  ./generate.sh
  ./build.sh
  UNREAL_GAMS_BUILD_RESULT=$?

else
  echo "NOT CHECKING UNREAL GAMS"
fi

if [ $GAMS -eq 1 ] || [ $EIGEN_AS_A_PREREQ -eq 1 ]; then

  cd $INSTALL_DIR

  echo "ENTERING $EIGEN_ROOT"
  if [ -d $EIGEN_ROOT ]; then
    (
      cd $EIGEN_ROOT
      if git branch | grep "* master"; then
        echo "EIGEN IN $EIGEN_ROOT IS MASTER BRANCH"
        echo "Deleting and redownloading stable release..."
        cd $HOME
        rm -rf $EIGEN_ROOT
      fi
    )
  fi
  if [ ! -d $EIGEN_ROOT ] ; then
    echo "git clone --single-branch --branch 3.3.7 --depth 1 https://github.com/eigenteam/eigen-git-mirror.git $EIGEN_ROOT"
    git clone --single-branch --branch 3.3.7 --depth 1 https://github.com/eigenteam/eigen-git-mirror.git $EIGEN_ROOT
    EIGEN_REPO_RESULT=$?
  else
    echo "UPDATING Eigen"
    cd $EIGEN_ROOT
    git pull
    EIGEN_REPO_RESULT=$?
  fi
else
  echo "NOT CHECKING EIGEN"
fi

if [ $CAPNP -eq 1 ] && [ $CAPNP_AS_A_PREREQ -eq 1 ]; then

  cd $INSTALL_DIR

  echo "ENTERING $CAPNP_ROOT"
  if [ -d $CAPNP_ROOT ]; then
    (
      cd $CAPNP_ROOT
      if git branch | grep "* master"; then
        echo "CAPNPROTO IN $CAPNP_ROOT IS MASTER BRANCH"
        echo "Deleting and redownloading stable release..."
        cd $HOME
        rm -rf $CAPNP_ROOT
      fi
    )
  fi
  if [ ! -d $CAPNP_ROOT ] ; then
    git clone --single-branch --branch release-0.7.0 --depth 1 https://github.com/capnproto/capnproto.git $CAPNP_ROOT
    CAPNP_REPO_RESULT=$?
  else
    cd $CAPNP_ROOT
    git pull
    CAPNP_REPO_RESULT=$?
  fi

  
  cd $CAPNP_ROOT/c++
  if [ $CLEAN -eq 1 ] ; then
    make clean -j $CORES
  fi


  autoreconf -i

  if [ $ANDROID -eq 1 ]; then
    export CROSS_PATH=${NDK_TOOLS}/bin/${ANDROID_TOOLCHAIN}
    export AR=${CROSS_PATH}-ar
    export AS=${CROSS_PATH}-as
    export NM=${CROSS_PATH}-nm
    export CC=${CROSS_PATH}-clang++
    export CXX=${CROSS_PATH}-clang++
    export LD=${CROSS_PATH}-ld
    export RANLIB=${CROSS_PATH}-ranlib
    export STRIP=${CROSS_PATH}-strip

    ./configure --host=${ANDROID_TOOLCHAIN} --with-sysroot=${SYSROOT} CPPFLAGS="${CPPLAGS} --sysroot=${SYSROOT} -I${SYSROOT}/usr/include -I${NDK_TOOLS}/include -std=c++11" LDFLAGS="${LDFLAGS} -L${SYSROOT}/usr/lib -L${NDK_TOOLS}/lib -L${NDK_TOOLS}/${ANDROID_TOOLCHAIN}/lib" LIBS='-lc++_shared'

  else 
    export PATH="$CAPNP_ROOT/c++:$PATH"
    export LD_LIBRARY_PATH="$CAPNP_ROOT/c++/.libs:$LD_LIBRARY_PATH"
    if [ $CLANG -ne 0 ] && [ $MAC -eq 0 ]; then
      if [ ! -z $FORCE_CC ] ; then
        export CC=$FORCE_CC
        echo "Forcing CC=$CC"
      else
        export CC=clang-6.0
      fi

      if [ ! -z $FORCE_CXX ] ; then
        export CXX=$FORCE_CXX
        echo "Forcing CXX=$CXX"
      else
        export CXX=clang++-6.0
      fi

      export CXXFLAGS="-stdlib=libc++ -I/usr/include/libcxxabi"
    fi

   ./configure

  fi

  make -j$CORES capnp capnpc-c++ libcapnp-json.la 
  CAPNP_BUILD_RESULT=$?

  if [ $JAVA -eq 1 ]; then
    if [ ! -d $CAPNPJAVA_ROOT ] ; then
      git clone https://github.com/capnproto/capnproto-java.git $CAPNPJAVA_ROOT

      CAPNPJAVA_REPO_RESULT=$?
      cd $CAPNPJAVA_ROOT
    else
      cd $CAPNPJAVA_ROOT
      git stash
      git pull
      CAPNPJAVA_REPO_RESULT=$?
    fi

    # the capnproto-java maintainer uses CAPNP_PREFIX within his Makefile
    if [ -z $CAPNP_PREFIX ] ; then
      export CAPNP_PREFIX=$CAPNP_ROOT
    fi

    if [[ ! $PATH =~ "$CAPNPJAVA_ROOT" ]]; then
      export PATH=$PATH:$CAPNPJAVA_ROOT
    fi

    patch -b -d $CAPNPJAVA_ROOT -p1 -i $GAMS_ROOT/scripts/linux/patches/capnpc-java.Makefile.patch
    make
    CAPNPJAVA_BUILD_RESULT=$?

  fi

  if [ ! -d $CAPNP_ROOT/c++/.libs ]; then
    echo "CapNProto is not built properly."
    exit 1;
  fi
  #Prereq if ends
else
  echo "NOT CHECKING CAPNPROTO"
fi

# this is common to Mac and Linux, so it needs to be outside of the above
if [ $PREREQS -eq 1 ]; then 
  if [ ! -d $OSC_ROOT ]; then 
    echo "Downloading oscpack"
    git clone https://github.com/jredmondson/oscpack.git $OSC_ROOT
  else
    echo "Updating oscpack"
    cd $OSC_ROOT
    git pull
  fi

  cd $OSC_ROOT
  echo "Cleaning oscpack"
  make clean
  echo "Building oscpack"
  if [ $MAC -eq 1 ]; then
    if [ $CLANG -eq 1 ]; then
      export CXX=clang++
    fi
  elif [ $CLANG -eq 1 ]; then
  
    if [ ! -z $FORCE_CC ] ; then
      export CC=$FORCE_CC
      echo "Forcing CC=$CC"
    else
      export CC=clang-6.0
    fi

    if [ ! -z $FORCE_CXX ] ; then
      export CXX=$FORCE_CXX
      echo "Forcing CXX=$CXX"
    else
      export CXX=clang++-6.0
    fi

  fi 

  if [ ! -z "$CXX" ]; then
    sudo make CXX=$CXX COPTS='-Wall -Wextra -fPIC' install
  else
    sudo make COPTS='-Wall -Wextra -fPIC' install
  fi

  if [ -d $SCRIMMAGE_GIT_ROOT ]; then
      cd $INSTALL_DIR
      rm -rf scrimmage
  fi
  
  echo "Installing SCRIMMAGE Dependencies"
  cd $INSTALL_DIR
  git clone https://github.com/gtri/scrimmage
  SCRIMMAGE_REPO_RESULT=$?
  
  cd scrimmage
  sudo ./setup/install-binaries.sh -e 0 -p 3
  sudo add-apt-repository ppa:kevin-demarco/scrimmage
  sudo apt-get update
  sudo apt-get install scrimmage-dependencies
  source /opt/scrimmage/*/setup.sh
  echo "SCRIMMAGE Dependencies Installed"

fi
  
if [ $ZMQ -eq 1 ]; then
  export ZMQ=1
  cd $INSTALL_DIR

  if [ -z $ZMQ_ROOT ]; then
      if [ $ANDROID -eq 1 ]; then
        export ZMQ_ROOT=$INSTALL_DIR/libzmq/output
      else 
        export ZMQ_ROOT=/usr/local
      fi
  fi

  echo "ZMQ_ROOT has been set to $ZMQ_ROOT"

  if [ ! -f $ZMQ_ROOT/lib/libzmq.so ]; then
    
     if [ ! -d libzmq ] ; then
       echo "git clone --depth 1 https://github.com/zeromq/libzmq"
       git clone --depth 1 https://github.com/zeromq/libzmq
       ZMQ_REPO_RESULT=$?
     fi
    
     #Go to zmq directory
     cd libzmq
    
     # For Android
     if [ $ANDROID -eq 1 ]; then 
        echo "UPDATING ZMQ FOR ANDROID"
        export ZMQ_OUTPUT_DIR=`pwd`/output
        ./autogen.sh && ./configure --enable-shared --disable-static --host=$ANDROID_TOOLCHAIN --prefix=$ZMQ_OUTPUT_DIR LDFLAGS="-L$ZMQ_OUTPUT_DIR/lib" CPPFLAGS="-fPIC -I$ZMQ_OUTPUT_DIR/include" LIBS="-lgcc"  CXX=$NDK_TOOLS/bin/$ANDROID_TOOLCHAIN-clang++ CC=$NDK_TOOLS/bin/$ANDROID_TOOLCHAIN-clang
        make clean install -j $CORES
        export ZMQ_ROOT=$ZMQ_OUTPUT_DIR
   
    #For regular builds.
     else 
       echo "UPDATING ZMQ"
       ./autogen.sh && ./configure && make clean -j $CORES
       make check
       sudo make install && sudo ldconfig
       ZMQ_BUILD_RESULT=$? 
       export ZMQ_ROOT=/usr/local
     fi
    
  fi
    
    #check again after installation
    if [ ! -f $ZMQ_ROOT/lib/libzmq.so ]; then
     echo "No libzmq found at $ZMQ_ROOT"
     exit 1;
    fi

#fi  #libzmq.so condition check ends.
else
  echo "NOT BUILDING ZEROMQ. If this is an error, delete the libzmq directory"
fi

if [ $SSL -eq 1 ]; then
  export SSL=1;
  if [ -z $SSL_ROOT ]; then
    if [ $MAC -eq 0 ]; then
      export SSL_ROOT=/usr
    elif [ $ANDROID -eq 1 ]; then
      export SSL_ROOT=$INSTALL_DIR/openssl
    else
      export SSL_ROOT=/usr/local/opt/openssl
    fi
  fi
fi

# set MADARA_ROOT since it is required by most other packages
if [ -z $MADARA_ROOT ] ; then
  export MADARA_ROOT=$INSTALL_DIR/madara
  echo "SETTING MADARA_ROOT to $MADARA_ROOT"
fi

# check if MADARA_ROOT/lib is in LD_LIBRARY_PATH and modify if needed
if [[ ! ":$LD_LIBRARY_PATH:" == *":$MADARA_ROOT/lib:"* ]]; then
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MADARA_ROOT/lib
fi

if [ $MADARA -eq 1 ] || [ $MADARA_AS_A_PREREQ -eq 1 ]; then

  echo "LD_LIBRARY_PATH for MADARA compile is $LD_LIBRARY_PATH"

  if [ $JAVA -eq 1 ] && [ -z $JAVA_HOME ]; then
     echo "Set JAVA_HOME or use prereqs to download Oracle Java automatically"
     exit 1;
  fi

  cd $INSTALL_DIR

  if [ ! -d $MADARA_ROOT ] ; then
  
    echo "DOWNLOADING MADARA"
    echo "git clone -b master --depth 1 https://github.com/jredmondson/madara.git $MADARA_ROOT"
    git clone -b master --depth 1 https://github.com/jredmondson/madara.git $MADARA_ROOT
    MADARA_REPO_RESULT=$?
  else

    cd $MADARA_ROOT
    if [ $MADARAPULL -eq 1 ]; then
      echo "UPDATING MADARA"
      git pull
      MADARA_REPO_RESULT=$?
    else
      echo "NOT PULLING MADARA"
    fi

    echo "CLEANING MADARA OBJECTS"
    if [ $CLEAN -eq 1 ] ; then
      make realclean -j $CORES
      rm GNUmakefile*
    fi

  fi

  if [ $NOKARL -eq 1 ] ; then
    echo "REMOVING KARL EXPRESSION EVALUATION FROM MADARA BUILD"
  fi

  if [ ! -z $FORCE_CC ] ; then
    export CC=$FORCE_CC
    echo "Forcing CC=$CC"
  fi

  if [ ! -z $FORCE_CXX ] ; then
    export CXX=$FORCE_CXX
    echo "Forcing CXX=$CXX"
  fi


  cd $MADARA_ROOT
  echo "GENERATING MADARA PROJECT"
  echo "perl $MPC_ROOT/mwc.pl -type make -features no_karl=$NOKARL,android=$ANDROID,python=$PYTHON,java=$JAVA,tests=$TESTS,tutorials=$TUTORIALS,docs=$DOCS,ssl=$SSL,zmq=$ZMQ,simtime=$SIMTIME,nothreadlocal=$NOTHREADLOCAL,clang=$CLANG,debug=$DEBUG,warnings=$WARNINGS,capnp=$CAPNP MADARA.mwc"
  perl $MPC_ROOT/mwc.pl -type make -features no_karl=$NOKARL,lz4=$LZ4,android=$ANDROID,python=$PYTHON,java=$JAVA,tests=$TESTS,tutorials=$TUTORIALS,docs=$DOCS,ssl=$SSL,zmq=$ZMQ,simtime=$SIMTIME,nothreadlocal=$NOTHREADLOCAL,clang=$CLANG,debug=$DEBUG,warnings=$WARNINGS,capnp=$CAPNP MADARA.mwc

  if [ $JAVA -eq 1 ]; then
    echo "DELETING MADARA JAVA CLASSES"
    # sometimes the jar'ing will occur before all classes are actually built when performing
    # multi-job builds, fix by deleting class files and recompiling with single build job
    find . -name "*.class" -delete
  fi

  echo "BUILDING MADARA"
  echo "make depend no_karl=$NOKARL android=$ANDROID capnp=$CAPNP java=$JAVA tests=$TESTS tutorials=$TUTORIALS docs=$DOCS ssl=$SSL zmq=$ZMQ simtime=$SIMTIME python=$PYTHON warnings=$WARNINGS -j $CORES"
  make depend no_karl=$NOKARL lz4=$LZ4 android=$ANDROID capnp=$CAPNP java=$JAVA tests=$TESTS tutorials=$TUTORIALS docs=$DOCS ssl=$SSL zmq=$ZMQ simtime=$SIMTIME python=$PYTHON warnings=$WARNINGS -j $CORES
  echo "make no_karl=$NOKARL android=$ANDROID capnp=$CAPNP java=$JAVA tests=$TESTS tutorials=$TUTORIALS docs=$DOCS ssl=$SSL zmq=$ZMQ simtime=$SIMTIME python=$PYTHON warnings=$WARNINGS -j $CORES"
  make no_karl=$NOKARL lz4=$LZ4 android=$ANDROID capnp=$CAPNP java=$JAVA tests=$TESTS tutorials=$TUTORIALS docs=$DOCS ssl=$SSL zmq=$ZMQ simtime=$SIMTIME python=$PYTHON warnings=$WARNINGS -j $CORES
  MADARA_BUILD_RESULT=$?
  if [ ! -f $MADARA_ROOT/lib/libMADARA.so ]; then
    MADARA_BUILD_RESULT=1
    echo -e "\e[91m MADARA library did not build properly. \e[39m"
    exit 1;
  fi

  if [ $STRIP -eq 1 ]; then
    echo "STRIPPING MADARA"
    $STRIP_EXE libMADARA.so*
  fi
else
  echo "NOT BUILDING MADARA"
fi

if [ $DMPL -eq 1 ] && [ ! -d $VREP_ROOT ]; then
  VREP_AS_A_PREREQ=1
fi

if [ $VREP -eq 1 ] || [ $VREP_AS_A_PREREQ -eq 1 ]; then
  if [ ! $VREP_ROOT ] ; then
    export VREP_ROOT=$INSTALL_DIR/vrep
    echo "SETTING VREP_ROOT to $VREP_ROOT"
  fi
  if [ ! -d $VREP_ROOT ]; then 
    cd $INSTALL_DIR
    echo "DOWNLOADING VREP"
    wget http://coppeliarobotics.com/files/$VREP_INSTALLER
    VREP_REPO_RESULT=$?
    mkdir $VREP_ROOT

    echo "UNPACKING VREP"
    tar xfz $VREP_INSTALLER -C $VREP_ROOT  --strip-components 1

    echo "CHANGING VREP OPTIONS"
    if [ -f $VREP_ROOT/system/usrset.txt ]; then
      for i in doNotShowOpenglSettingsMessage doNotShowCrashRecoveryMessage doNotShowUpdateCheckMessage; do
        cat $VREP_ROOT/system/usrset.txt | sed "s/$i = false/$i = true/g" > $VREP_ROOT/system/usrset.txt1
        mv $VREP_ROOT/system/usrset.txt1 $VREP_ROOT/system/usrset.txt
      done
    else
      for i in doNotShowOpenglSettingsMessage doNotShowCrashRecoveryMessage doNotShowUpdateCheckMessage; do
        echo "$i = true" >> $VREP_ROOT/system/usrset.txt
      done
    fi

    echo "CONFIGURING 20 VREP PORTS"
    $GAMS_ROOT/scripts/simulation/vrep/remoteApiConnectionsGen.pl 19905 20


    echo "PATCHING VREP"
    patch -b -d $VREP_ROOT -p1 -i $GAMS_ROOT/scripts/linux/patches/00_VREP_extApi_readPureDataFloat_alignment.patch
  else
    echo "NO CHANGE TO VREP"
  fi
else
  echo "NOT DOWNLOADING VREP"
fi

if [ $DMPL -eq 1 ] && [ ! -d $GAMS_ROOT ]; then
  GAMS_AS_A_PREREQ=1
fi

echo "LD_LIBRARY_PATH for GAMS compile is $LD_LIBRARY_PATH"


# Install SCRIMMAGE after GAMS because GAMS_ROOT needs to be set for the plugin path. Except, SCRIMMAGE has to be built before GAMS otherwise GAMS can't find some headers brought in by SCRIMMAGE setup files.
if [ $SCRIMMAGE -eq 1 ] || [ $SCRIMMAGE_AS_A_PREREQ -eq 1 ]; then
  if [ ! $SCRIMMAGE_GIT_ROOT ] ; then
      export SCRIMMAGE_GIT_ROOT="$INSTALL_DIR/scrimmage"
      echo "SETTING SCRIMMAGE_GIT_ROOT to $SCRIMMAGE_GIT_ROOT"
  fi
  
  if [ -d $SCRIMMAGE_GIT_ROOT ]; then
      cd $INSTALL_DIR/scrimmage
      echo "BUILDING SCRIMMAGE. It SHOULD BE cloned already from the PREREQS section."
      
      if [ ! -d "$SCRIMMAGE_GIT_ROOT/build" ]; then
        mkdir build 
      fi
       
        cd build
        cmake ..
        make
        source $HOME/.scrimmage/setup.bash
        export SCRIMMAGE_ROOT=$SCRIMMAGE_ROOT
        export SCRIMMAGE_GIT_ROOT="$INSTALL_DIR/scrimmage"

        # Order is important actually, if scrimmage setup.bash is AFTER gams/env.sh, then the SCRIMMAGE_PLUGIN_PATH and SCRIMMAGE_MISSION_PATH will be incorrectly set because scrimmage/setup.bash sets it wrong.
        if ! grep -q "$HOME/.scrimmage/setup.bash" $HOME/.bashrc ; then
            echo "source $HOME/.scrimmage/setup.bash" >> ~/.bashrc
        fi

        echo "SCRIMMAGE BUILT! Ready to use with GAMS."
        
  else
      echo -e "\e[91Something happened between setting up the PREREQS for SCRIMMAGE and now. Please remove any scrimmage directories and re-install the prereqs then re-run with the scrimmage-gams parameter [39m" 
  fi
fi

# if gams has been specified, or if dmpl is specified and GAMS_ROOT doesn't exist
if [ $GAMS -eq 1 ] || [ $GAMS_AS_A_PREREQ -eq 1 ]; then

  # build GAMS
  if [ -z $GAMS_ROOT ] ; then
    # Update Mar 6 2020: If the current directory is named gams, then it will not reinstall a new GAMS as it currently does. That caused problems when developing from any other repo than https://github.com/jredmondson/gams
    CUR_DUR_NAME=${PWD##*/}
    if [ ! "gams" -eq $CUR_DUR_NAME ]; then
        export GAMS_ROOT=$INSTALL_DIR/gams
        echo "SETTING GAMS_ROOT to $GAMS_ROOT"
    else 
        export GAMS_ROOT=$INSTALL_DIR
        echo "SETTING GAMS_ROOT to $GAMS_ROOT"
    fi
  fi
  if [ ! -d $GAMS_ROOT ] ; then
    echo "DOWNLOADING GAMS"
    echo "git clone -b master --depth 1 --single-branch https://github.com/jredmondson/gams.git $GAMS_ROOT"
    git clone -b master --depth 1 --single-branch https://github.com/jredmondson/gams.git $GAMS_ROOT
    GAMS_REPO_RESULT=$?
    
  else
    cd $GAMS_ROOT

    if [ $GAMSPULL -eq 1 ]; then
      echo "UPDATING GAMS"
      git pull
      GAMS_REPO_RESULT=$?
    else
      echo "NOT PULLING GAMS"
    fi

    echo "CLEANING GAMS OBJECTS"
    if [ $CLEAN -eq 1 ] ; then
      make realclean -j $CORES
      rm GNUmakefile*
    fi

  fi

  if [ $NOKARL -eq 1 ] ; then
    echo "REMOVING KARL EXPRESSION EVALUATION FROM GAMS BUILD"
  fi

  if [ ! -z $FORCE_CC ] ; then
    export CC=$FORCE_CC
    echo "Forcing CC=$CC"
  fi

  if [ ! -z $FORCE_CXX ] ; then
    export CXX=$FORCE_CXX
    echo "Forcing CXX=$CXX"
  fi

  cd $GAMS_ROOT

  echo "GENERATING GAMS PROJECT"
  echo "perl $MPC_ROOT/mwc.pl -type make -features no_karl=$NOKARL,airlib=$AIRLIB,java=$JAVA,ros=$ROS,types=$TYPES,capnp=$CAPNP,vrep=$VREP,scrimmage=$SCRIMMAGE,tests=$TESTS,android=$ANDROID,docs=$DOCS,clang=$CLANG,simtime=$SIMTIME,debug=$DEBUG,warnings=$WARNINGS gams.mwc"
  perl $MPC_ROOT/mwc.pl -type make -features no_karl=$NOKARL,airlib=$AIRLIB,java=$JAVA,ros=$ROS,python=$PYTHON,types=$TYPES,capnp=$CAPNP,vrep=$VREP,scrimmage=$SCRIMMAGE,tests=$TESTS,android=$ANDROID,docs=$DOCS,clang=$CLANG,simtime=$SIMTIME,debug=$DEBUG,warnings=$WARNINGS gams.mwc

  if [ $TYPES -eq 1 ]; then
    # Strip the unnecessary NOTPARALLEL: directives
    sed -i '/\.NOTPARALLEL:/d' Makefile.types
  fi

  if [ $JAVA -eq 1 ]; then
    # sometimes the jar'ing will occur before all classes are actually built when performing
    # multi-job builds, fix by deleting class files and recompiling with single build job
    find . -name "*.class" -delete
  fi

  echo "BUILDING GAMS"
  echo "make depend no_karl=$NOKARL airlib=$AIRLIB capnp=$CAPNP java=$JAVA ros=$ROS types=$TYPES vrep=$VREP scrimmage=$SCRIMMAGE tests=$TESTS android=$ANDROID simtime=$SIMTIME docs=$DOCS warnings=$WARNINGS -j $CORES"
  make depend no_karl=$NOKARL airlib=$AIRLIB capnp=$CAPNP java=$JAVA ros=$ROS types=$TYPES vrep=$VREP scrimmage=$SCRIMMAGE tests=$TESTS android=$ANDROID simtime=$SIMTIME docs=$DOCS warnings=$WARNINGS -j $CORES
  echo "make no_karl=$NOKARL airlib=$AIRLIB capnp=$CAPNP java=$JAVA ros=$ROS types=$TYPES vrep=$VREP scrimmage=$SCRIMMAGE tests=$TESTS android=$ANDROID simtime=$SIMTIME docs=$DOCS warnings=$WARNINGS -j $CORES"
  make no_karl=$NOKARL airlib=$AIRLIB capnp=$CAPNP java=$JAVA ros=$ROS types=$TYPES vrep=$VREP scrimmage=$SCRIMMAGE python=$PYTHON tests=$TESTS android=$ANDROID simtime=$SIMTIME docs=$DOCS warnings=$WARNINGS -j $CORES
  GAMS_BUILD_RESULT=$?
  
  if [ ! -f $GAMS_ROOT/lib/libGAMS.so ]; then
    GAMS_BUILD_RESULT=1
    echo -e "\e[91mGAMS library did not build properly\e[39m";
    exit 1;
  fi

  if [ $STRIP -eq 1 ]; then
    echo "STRIPPING GAMS"
    $STRIP_EXE libGAMS.so*
  fi
#  if [ $ANDROID -eq 1 ]; then
#     echo "Building Demo Android app"
#     cd $GAMS_ROOT/port/android
#     chmod a+x build.sh
#     ./build.sh
#    cd $GAMS_ROOT
#   fi
else
  echo "NOT BUILDING GAMS"
fi

# Install SCRIMMAGE after GAMS because GAMS_ROOT needs to be set for the plugin path 
if [ $SCRIMMAGE -eq 1 ] || [ $SCRIMMAGE_AS_A_PREREQ -eq 1 ]; then
  if [ ! $SCRIMMAGE_GIT_ROOT ] ; then
      export SCRIMMAGE_GIT_ROOT=$INSTALL_DIR/scrimmage
      echo "SETTING SCRIMMAGE_GIT_ROOT to $SCRIMMAGE_GIT_ROOT"
  fi

  if [ -d $SCRIMMAGE_GIT_ROOT ]; then
      cd $INSTALL_DIR/scrimmage
      echo "BUILDING SCRIMMAGE. It SHOULD BE cloned already from the PREREQS section."
      
      if [ ! -d "$SCRIMMAGE_GIT_ROOT/build" ]; then
        mkdir build 
      fi
       
        cd build
        cmake ..
        make
        source ~/.scrimmage/setup.bash
        echo "export SCRIMMAGE_PLUGIN_PATH=$SCRIMMAGE_PLUGIN_PATH:$GAMS_ROOT/lib/scrimmage_plugins" >> $HOME/.gams/env.sh
        export SCRIMMAGE_PLUGIN_PATH=$SCRIMMAGE_PLUGIN_PATH:$GAMS_ROOT/lib/scrimmage_plugins

        # Order is important actually, if scrimmage setup.bash is AFTER gams/env.sh, then the SCRIMMAGE_PLUGIN_PATH and SCRIMMAGE_MISSION_PATH will be incorrectly set because scrimmage/setup.bash sets it wrong.
        if ! grep -q ".scrimmage/setup.bash" $HOME/.bashrc ; then
        echo "source ~/.scrimmage/setup.bash" >> ~/.bashrc
        fi

        echo "SCRIMMAGE BUILT! Ready to use with GAMS."
        
  else
      echo -e "\e[91Something happened between setting up the PREREQS for SCRIMMAGE and now. Please remove any scrimmage directories and re-install the prereqs then re-run with the scrimmage-gams parameter [39m" 
  fi
fi
 
# check if GAMS_ROOT/lib is in LD_LIBRARY_PATH and modify if needed
if [[ ! ":$LD_LIBRARY_PATH:" == *":$GAMS_ROOT/lib:"* ]]; then
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GAMS_ROOT/lib
fi
 

if [ $DMPL -eq 1 ]; then

  echo "LD_LIBRARY_PATH for DMPLC compile is $LD_LIBRARY_PATH"

  cd $INSTALL_DIR

  if [ ! $VREP_ROOT ] ; then
    export VREP_ROOT=$INSTALL_DIR/vrep
    echo "SETTING VREP_ROOT to $VREP_ROOT"
  fi

  # build DART
  if [ -z $DMPL_ROOT ] ; then
    export DMPL_ROOT=$INSTALL_DIR/dmplc
    echo "SETTING DMPL_ROOT to $DMPL_ROOT"
  fi
  if [ ! -d $DMPL_ROOT ] ; then
    echo "DOWNLOADING DMPL"
    git clone --depth 1 -b release-0.4.0 https://github.com/cps-sei/dmplc.git $DMPL_ROOT
    DART_REPO_RESULT=$?
    
  else
    echo "UPDATING DMPL"
    cd $DMPL_ROOT
    git pull

    echo "CLEANING DMPL OBJECTS"
    if [ $CLEAN -eq 1]; then
      make clean -j $CORES
    fi

  fi

  cd $DMPL_ROOT
  make depend MZSRM=0 -j $CORES
  make MZSRM=0 -j $CORES
  DART_BUILD_RESULT=$?
fi

if [ $ANDROID_TESTS -eq 1 ]; then
  cd $GAMS_ROOT/port/android
  ./run-middleware-tests.sh
fi

if [ $CAPNP_JAVA -eq 1 ]; then

	export BASE_CAPNP_DIR=$INSTALL_DIR/capnproto-java
	export CAPNP_PREFIX=$BASE_CAPNP_DIR/capnproto
	export CAPNP_JAVA_DIR=$BASE_CAPNP_DIR/capnproto-java
	if [ $CLEAN = 1 ]; then
		rm -rf $BASE_CAPNP_DIR 
		mkdir -p $BASE_CAPNP_DIR
		cd $BASE_CAPNP_DIR
		git clone --single-branch --branch release-0.6.1 --depth 1 https://github.com/capnproto/capnproto.git
		git clone https://github.com/capnproto/capnproto-java.git 
	fi
	#Build CAPNP Dir.
	cd $CAPNP_PREFIX/c++
	autoreconf -i
	./configure
	make -j4
	export PATH=$CAPNP_PREFiX/c++:$PATH
	export CAPNP_CXX_FLAGS='-I $(CAPNP_PREFIX)/c++/src -L $(CAPNP_PREFIX)/c++/.libs -lkj -lcapnp'
	#Build Capnp java
	cd $CAPNP_JAVA_DIR/cmake/
	cmake -DCAPNP_PKG_PATH=$CAPNP_PREFIX/c++/capnp.pc $CAPNP_PREFIX/c++/CMakeLists.txt
	cd $CAPNP_PREFIX/c++
	make -j4
	cd $CAPNP_JAVA_DIR
	sed -i -e 's/c++11/c++14/g' $CAPNP_JAVA_DIR/Makefile
	sed -i '/CAPNP_CXX_FLAGS=\$/d' $CAPNP_JAVA_DIR/Makefile
	make -j4

fi

if [ $VREP_CONFIG -eq 1 ]; then
  echo "CONFIGURING 20 VREP PORTS"
  $GAMS_ROOT/scripts/simulation/vrep/remoteApiConnectionsGen.pl 19905 20
fi

# create example config files with default GAMS multicast IPs
if [ ! -d $HOME/.madara ]; then
  mkdir $HOME/.madara
  cp $SCRIPTS_DIR/../common/README.txt $HOME/.madara
fi

# save the last feature changing build (need to fix this by flattening $@)
if [ $CLEAN -eq 1 ]; then
  echo "$@" > $GAMS_ROOT/last_build.lst
  
fi

echo ""
echo "BUILD STATUS"


if [ $MPC -eq 1 ] || [ $MPC_AS_A_PREREQ -eq 1 ]; then
  echo "  MPC"
  if [ $MPC_REPO_RESULT -eq 0 ]; then
    echo -e "    REPO=\e[92mPASS\e[39m"
  else
    echo -e "    REPO=\e[91mFAIL\e[39m"
    (( BUILD_ERRORS++ ))
  fi
fi

if [ $LZ4 -eq 1 ]; then
  echo "  LZ4"
  if [ $LZ4_REPO_RESULT -eq 0 ]; then
    echo -e "    REPO=\e[92mPASS\e[39m"
  else
    echo -e "    REPO=\e[91mFAIL\e[39m"
    (( BUILD_ERRORS++ ))
  fi
fi

if [ $ZMQ -eq 1 ]; then
  echo "  ZMQ"
  if [ $ZMQ_REPO_RESULT -eq 0 ]; then
    echo -e "    REPO=\e[92mPASS\e[39m"
  else
    echo -e "    REPO=\e[91mFAIL\e[39m"
    (( BUILD_ERRORS++ ))
  fi
  if [ $ZMQ_BUILD_RESULT -eq 0 ]; then
    echo -e "    BUILD=\e[92mPASS\e[39m"
  else
    echo -e "    BUILD=\e[91mFAIL\e[39m"
    # MAC has multiple failed tests for ZeroMQ
    if [ $MAC -eq 0 ]; then
      (( BUILD_ERRORS++ ))
    fi
  fi
fi

if [ $VREP -eq 1 ]; then
  echo "  VREP"
  if [ $VREP_REPO_RESULT -eq 0 ]; then
    echo -e "    REPO=\e[92mPASS\e[39m"
  else
    echo -e "    REPO=\e[91mFAIL\e[39m"
    (( BUILD_ERRORS++ ))
  fi
fi

if [ $MADARA -eq 1 ] || [ $MADARA_AS_A_PREREQ -eq 1 ]; then
  if [ $CAPNP -eq 1 ]; then
    echo "  CAPNPROTO"
    if [ $CAPNP_REPO_RESULT -eq 0 ]; then
      echo -e "    REPO=\e[92mPASS\e[39m"
    else
      echo -e "    REPO=\e[91mFAIL\e[39m"
      (( BUILD_ERRORS++ ))
    fi
    if [ $CAPNP_BUILD_RESULT -eq 0 ]; then
      echo -e "    BUILD=\e[92mPASS\e[39m"
    else
      echo -e "    BUILD=\e[91mFAIL\e[39m"
      (( BUILD_ERRORS++ ))
    fi

    if [ $JAVA -eq 1 ]; then
      echo "  CAPNPROTO-JAVA"
      if [ $CAPNPJAVA_REPO_RESULT -eq 0 ]; then
        echo -e "    REPO=\e[92mPASS\e[39m"
      else
        echo -e "    REPO=\e[91mFAIL\e[39m"
        (( BUILD_ERRORS++ ))
      fi
      if [ $CAPNPJAVA_BUILD_RESULT -eq 0 ]; then
        echo -e "    BUILD=\e[92mPASS\e[39m"
      else
        echo -e "    BUILD=\e[91mFAIL\e[39m"
        (( BUILD_ERRORS++ ))
      fi
    fi
  fi


  echo "  MADARA"

  if [ $MADARAPULL -eq 1 ]; then
    if [ $MADARA_REPO_RESULT -eq 0 ]; then
      echo -e "    REPO=\e[92mPASS\e[39m"
    else
      echo -e "    REPO=\e[91mFAIL\e[39m"
      (( BUILD_ERRORS++ ))
    fi
  fi

  if [ $MADARA_BUILD_RESULT -eq 0 ]; then
    echo -e "    BUILD=\e[92mPASS\e[39m"
  else
    echo -e "    BUILD=\e[91mFAIL\e[39m"
    (( BUILD_ERRORS++ ))
  fi
fi

if [ $UNREAL -eq 1 ] && [ $FORCE_UNREAL -eq 1 ]; then
  echo "  UNREAL"
  if [ $UNREAL_BUILD_RESULT -eq 0 ]; then
    echo -e "    BUILD=\e[92mPASS\e[39m"
  else
    echo -e "    BUILD=\e[91mFAIL\e[39m"
    (( BUILD_ERRORS++ ))
  fi
fi

if [ $AIRLIB -eq 1 ] && [ $FORCE_AIRSIM -eq 1 ]; then
  echo "  AIRSIM"
  if [ $AIRSIM_BUILD_RESULT -eq 0 ]; then
    echo -e "    BUILD=\e[92mPASS\e[39m"
  else
    echo -e "    BUILD=\e[91mFAIL\e[39m"
    (( BUILD_ERRORS++ ))
  fi
fi

if [ $GAMS -eq 1 ] || [ $GAMS_AS_A_PREREQ -eq 1 ]; then
  echo "  EIGEN"
  if [ $EIGEN_REPO_RESULT -eq 0 ]; then
    echo -e "    REPO=\e[92mPASS\e[39m"
  else
    echo -e "    REPO=\e[91mFAIL\e[39m"
    (( BUILD_ERRORS++ ))
  fi

  echo "  GAMS"
  if [ $GAMSPULL -eq 1 ]; then
    if [ $GAMS_REPO_RESULT -eq 0 ]; then
      echo -e "    REPO=\e[92mPASS\e[39m"
    else
      echo -e "    REPO=\e[91mFAIL\e[39m"
      (( BUILD_ERRORS++ ))
    fi
  fi

  if [ $GAMS_BUILD_RESULT -eq 0 ]; then
    echo -e "    BUILD=\e[92mPASS\e[39m"
  else
    echo -e "    BUILD=\e[91mFAIL\e[39m"
    (( BUILD_ERRORS++ ))
  fi
fi

if [ $UNREAL_GAMS -eq 1 ] ; then
  echo "  UNREAL GAMS"
  if [ $UNREAL_GAMS_REPO_RESULT -eq 0 ]; then
    echo -e "    REPO=\e[92mPASS\e[39m"
  else
    echo -e "    REPO=\e[91mFAIL\e[39m"
    (( BUILD_ERRORS++ ))
  fi
  if [ $UNREAL_GAMS_BUILD_RESULT -eq 0 ]; then
    echo -e "    BUILD=\e[92mPASS\e[39m"
  else
    echo -e "    BUILD=\e[91mFAIL\e[39m"
    (( BUILD_ERRORS++ ))
  fi
fi

echo -e ""
echo -e "Saving environment variables into \$HOME/.gams/env.sh"

if grep -q "export MPC_ROOT" $HOME/.gams/env.sh ; then
  sed -i 's@MPC_ROOT=.*@MPC_ROOT='"$MPC_ROOT"'@' $HOME/.gams/env.sh
else
  echo "export MPC_ROOT=$MPC_ROOT" >> $HOME/.gams/env.sh
fi

if grep -q "export EIGEN_ROOT" $HOME/.gams/env.sh ; then
  sed -i 's@EIGEN_ROOT=.*@EIGEN_ROOT='"$EIGEN_ROOT"'@' $HOME/.gams/env.sh
else
  echo "export EIGEN_ROOT=$EIGEN_ROOT" >> $HOME/.gams/env.sh
fi

if grep -q "export CAPNP_ROOT" $HOME/.gams/env.sh ; then
  sed -i 's@CAPNP_ROOT=.*@CAPNP_ROOT='"$CAPNP_ROOT"'@' $HOME/.gams/env.sh
else
  echo "export CAPNP_ROOT=$CAPNP_ROOT" >> $HOME/.gams/env.sh
fi

if grep -q "export MADARA_ROOT" $HOME/.gams/env.sh ; then
  sed -i 's@MADARA_ROOT=.*@MADARA_ROOT='"$MADARA_ROOT"'@' $HOME/.gams/env.sh
else
  echo "export MADARA_ROOT=$MADARA_ROOT" >> $HOME/.gams/env.sh
fi

if grep -q "export GAMS_ROOT" $HOME/.gams/env.sh ; then
  sed -i 's@export GAMS_ROOT=.*@export GAMS_ROOT='"$GAMS_ROOT"'@' $HOME/.gams/env.sh
else
  echo "export GAMS_ROOT=$GAMS_ROOT" >> $HOME/.gams/env.sh
fi

if [ $SSL -eq 1 ]; then
  if [ -z $SSL_ROOT ]; then
    export SSL_ROOT=/usr
  fi

  if grep -q SSL_ROOT $HOME/.gams/env.sh ; then
    sed -i 's@SSL_ROOT=.*@SSL_ROOT='"$SSL_ROOT"'@' $HOME/.gams/env.sh
  else
    echo "export SSL_ROOT=$SSL_ROOT" >> $HOME/.gams/env.sh
  fi

fi

if [ $LZ4 -eq 1 ]; then

  if grep -q LZ4_ROOT $HOME/.gams/env.sh ; then
    sed -i 's@LZ4_ROOT=.*@LZ4_ROOT='"$LZ4_ROOT"'@' $HOME/.gams/env.sh
  else
    echo "export LZ4_ROOT=$LZ4_ROOT" >> $HOME/.gams/env.sh
  fi

fi

if [ $ZMQ -eq 1 ]; then

  if grep -q ZMQ_ROOT $HOME/.gams/env.sh ; then
    sed -i 's@ZMQ_ROOT=.*@ZMQ_ROOT='"$ZMQ_ROOT"'@' $HOME/.gams/env.sh
  else
    echo "export ZMQ_ROOT=$ZMQ_ROOT" >> $HOME/.gams/env.sh
  fi
fi

if [ $JAVA -eq 1 ]; then

  if grep -q JAVA_HOME $HOME/.gams/env.sh ; then
    sed -i 's@JAVA_HOME=.*@JAVA_HOME='"$JAVA_HOME"'@' $HOME/.gams/env.sh
  else
    echo "export JAVA_HOME=$JAVA_HOME" >> $HOME/.gams/env.sh
  fi

fi

if grep -q "export DMPL_ROOT" $HOME/.gams/env.sh ; then
  sed -i 's@export DMPL_ROOT=.*@export DMPL_ROOT='"$DMPL_ROOT"'@' $HOME/.gams/env.sh
else
  echo "export DMPL_ROOT=$DMPL_ROOT" >> $HOME/.gams/env.sh
fi

if grep -q "export PYTHONPATH" $HOME/.gams/env.sh ; then
  sed -i 's@export PYTHONPATH=.*@export PYTHONPATH='"\$PYTHONPATH"':'"\$MADARA_ROOT/lib"':'"\$GAMS_ROOT/lib"'@' $HOME/.gams/env.sh
else
  echo "export PYTHONPATH=\$PYTHONPATH:\$MADARA_ROOT/lib:\$GAMS_ROOT/lib" >> $HOME/.gams/env.sh
fi

if grep -q "export CAPNPJAVA_ROOT" $HOME/.gams/env.sh ; then
  sed -i 's@export CAPNPJAVA_ROOT=.*@export CAPNPJAVA_ROOT='"$CAPNPJAVA_ROOT"'@' $HOME/.gams/env.sh
else
  echo "export CAPNPJAVA_ROOT=$CAPNPJAVA_ROOT" >> $HOME/.gams/env.sh
fi


if [ $ANDROID -eq 1 ]; then
  if grep -q NDK_ROOT $HOME/.gams/env.sh ; then
    sed -i 's@NDK_ROOT=.*@NDK_ROOT='"$NDK_ROOT"'@' $HOME/.gams/env.sh
    sed -i 's@NDK_TOOLS=.*@NDK_TOOLS='"$NDK_TOOLS"'@' $HOME/.gams/env.sh
    sed -i 's@SYSROOT=.*@SYSROOT='"$SYSROOT"'@' $HOME/.gams/env.sh
    sed -i 's@ANDROID_ARCH=.*@ANDROID_ARCH='"$ANDROID_ARCH"'@' $HOME/.gams/env.sh
    sed -i 's@BOOST_ANDROID_ROOT=.*@BOOST_ANDROID_ROOT='"$BOOST_ANDROID_ROOT"'@' $HOME/.gams/env.sh
  else
    echo "export NDK_ROOT=$NDK_ROOT" >> $HOME/.gams/env.sh
    echo "export NDK_TOOLS=$NDK_TOOLS" >> $HOME/.gams/env.sh
    echo "export SYSROOT=$SYSROOT" >> $HOME/.gams/env.sh
    echo "export ANDROID_ARCH=$ANDROID_ARCH" >> $HOME/.gams/env.sh
    echo "export BOOST_ANDROID_ROOT=$BOOST_ANDROID_ROOT" >> $HOME/.gams/env.sh
  fi

fi

if [ $MAC -eq 0 ]; then

  if grep -q LD_LIBRARY_PATH $HOME/.gams/env.sh ; then
    sed -i 's@LD_LIBRARY_PATH=.*@LD_LIBRARY_PATH='"\$LD_LIBRARY_PATH"':'"\$MADARA_ROOT/lib"':'"\$GAMS_ROOT/lib"':'"\$VREP_ROOT"':'"\$CAPNP_ROOT/c++/.libs"'@' $HOME/.gams/env.sh
  else
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$MADARA_ROOT/lib:\$GAMS_ROOT/lib:\$VREP_ROOT:\$CAPNP_ROOT/c++/.libs" >> $HOME/.gams/env.sh
  fi

else

  if grep -q DYLD_LIBRARY_PATH $HOME/.gams/env.sh ; then
    sed -i 's@DYLD_LIBRARY_PATH=.*@DYLD_LIBRARY_PATH='"\$DYLD_LIBRARY_PATH"':'"\$MADARA_ROOT/lib"':'"\$GAMS_ROOT/lib"':'"\$VREP_ROOT"':'"\$CAPNP_ROOT/c++/.libs"'@' $HOME/.gams/env.sh
  else
    echo "export DYLD_LIBRARY_PATH=\$DYLD_LIBRARY_PATH:\$MADARA_ROOT/lib:\$GAMS_ROOT/lib:\$VREP_ROOT:\$CAPNP_ROOT/c++/.libs" >> $HOME/.gams/env.sh
  fi

fi

if grep -q "export PATH" $HOME/.gams/env.sh ; then
  sed -i 's@export PATH=.*@export PATH='"\$PATH"':'"\$MPC_ROOT"':'"\$VREP_ROOT"':'"\$CAPNP_ROOT/c++"':'"\$MADARA_ROOT/bin"':'"\$GAMS_ROOT/bin"':'"\$DMPL_ROOT/src/DMPL"':'"\$DMPL_ROOT/src/vrep"':'"\$CAPNPJAVA_ROOT"'@' $HOME/.gams/env.sh
else
  echo "export PATH=\$PATH:\$MPC_ROOT:\$VREP_ROOT:\$CAPNP_ROOT/c++:\$MADARA_ROOT/bin:\$GAMS_ROOT/bin:\$DMPL_ROOT/src/DMPL:\$DMPL_ROOT/src/vrep:\$CAPNPJAVA_ROOT" >> $HOME/.gams/env.sh
fi

# No check if it's unset. the source .scrimmage/setup.bash script always clears the plugin path var and sets it to the base one. We just have to append to it. Other option is make user do it manually. I don't think this is harmful.
echo "export SCRIMMAGE_PLUGIN_PATH=$SCRIMMAGE_PLUGIN_PATH:$GAMS_ROOT/lib/scrimmage_plugins" >> $HOME/.gams/env.sh
echo "export SCRIMMAGE_MISSION_PATH=$SCRIMMAGE_MISSION_PATH:$GAMS_ROOT/src/gams/platforms/scrimmage/missions/" >> $HOME/.gams/env.sh

if ! grep -q ".gams/env.sh" $HOME/.bashrc ; then
  echo "Updating bashrc to load environment. Close terminals to reload."
  echo ""
  echo "" >> $HOME/.bashrc
  echo "source \$HOME/.gams/env.sh" >> $HOME/.bashrc
else
  echo "If environment has changed, close terminals or reload .bashrc"
  echo ""
fi

echo "Sourcing final GAMS environment"
source $HOME/.gams/env.sh

echo "BUILD_ERRORS=$BUILD_ERRORS"
exit $BUILD_ERRORS
