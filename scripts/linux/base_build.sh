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

DEBUG=0
TESTS=0
VREP=0
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
SSL=0
DMPL=0
LZ4=0
PYTHON=0
CLEAN=1
MAC=${MAC:-0}
BUILD_ERRORS=0

MPC_DEPENDENCY_ENABLED=0
MADARA_DEPENDENCY_ENABLED=0
MPC_AS_A_PREREQ=0
MADARA_AS_A_PREREQ=0
VREP_AS_A_PREREQ=0
GAMS_AS_A_PREREQ=0
EIGEN_AS_A_PREREQ=0
CAPNP_AS_A_PREREQ=0

MPC_REPO_RESULT=0
DART_REPO_RESULT=0
DART_BUILD_RESULT=0
GAMS_REPO_RESULT=0
GAMS_BUILD_RESULT=0
MADARA_REPO_RESULT=0
MADARA_BUILD_RESULT=0
MPC_REPO_RESULT=0
VREP_REPO_RESULT=0
ZMQ_REPO_RESULT=0
ZMQ_BUILD_RESULT=0
LZ4_REPO_RESULT=0
CAPNP_REPO_RESULT=0
CAPNP_BUILD_RESULT=0

STRIP_EXE=strip
VREP_INSTALLER="V-REP_PRO_EDU_V3_4_0_Linux.tar.gz"
INSTALL_DIR=`pwd`
SCRIPTS_DIR=`dirname $0`

if [ -z $CORES ] ; then
  echo "CORES unset, so setting it to default of 1"
  echo "  If you have more than one CPU core, try export CORES=<num cores>"
  echo "  CORES=1 (the default) will be much slower than CORES=<num cores>"
  export CORES=1  
fi

for var in "$@"
do
  if [ "$var" = "tests" ]; then
    TESTS=1
  elif [ "$var" = "clean" ]; then
    CLEAN=1
  elif [ "$var" = "debug" ]; then
    DEBUG=1
  elif [ "$var" = "docs" ]; then
    DOCS=1
  elif [ "$var" = "prereqs" ]; then
    PREREQS=1
  elif [ "$var" = "vrep" ]; then
    VREP=1
  elif [ "$var" = "vrep-config" ]; then
    VREP_CONFIG=1
  elif [ "$var" = "java" ]; then
    JAVA=1
  elif [ "$var" = "clang" ]; then
    CLANG=1
  elif [ "$var" = "ros" ]; then
    ROS=1
  elif [ "$var" = "mpc" ]; then
    MPC=1
  elif [ "$var" = "madara" ]; then
    MADARA=1
  elif [ "$var" = "noclean" ]; then
    CLEAN=0
  elif [ "$var" = "python" ]; then
    PYTHON=1
  elif [ "$var" = "gams" ]; then
    GAMS=1
  elif [ "$var" = "dart" ]; then
    DMPL=1
  elif [ "$var" = "dmpl" ]; then
    DMPL=1
  elif [ "$var" = "lz4" ]; then
    LZ4=1
  elif [ "$var" = "zmq" ]; then
    ZMQ=1
  elif [ "$var" = "simtime" ]; then
    SIMTIME=1
  elif [ "$var" = "ssl" ]; then
    SSL=1
  elif [ "$var" = "odroid" ]; then
    ODROID=1
    STRIP_EXE=${LOCAL_CROSS_PREFIX}strip
  elif [ "$var" = "android" ]; then
    ANDROID=1
    JAVA=1
    STRIP_EXE=${LOCAL_CROSS_PREFIX}strip
  elif [ "$var" = "strip" ]; then
    STRIP=1
  else
    echo "Invalid argument: $var"
    echo "  args can be zero or more of the following, space delimited"
    echo "  debug           create a debug build, with minimal optimizations"
    echo "  mpc             download MPC if prereqs is enabled"
    echo "  android         build android libs, turns on java"
    echo "  clang           build using clang++-5.0 and libc++"
    echo "  clean           run 'make clean' before builds (default)"
    echo "  dmpl            build DART DMPL verifying compiler"
    echo "  docs            generate API documentation"
    echo "  gams            build GAMS"
    echo "  java            build java jar"
    echo "  lz4             build with LZ4 compression"
    echo "  madara          build MADARA"
    echo "  noclean         do not run 'make clean' before builds"
    echo "  odroid          target ODROID computing platform"
    echo "  python          build with Python 2.7 support"
    echo "  prereqs         use apt-get to install prereqs"
    echo "  ros             build ROS platform classes"
    echo "  ssl             build with OpenSSL support"
    echo "  strip           strip symbols from the libraries"
    echo "  tests           build test executables"
    echo "  vrep            build with vrep support"
    echo "  vrep-config     configure vrep to support up to 20 agents"
    echo "  zmq             build with ZeroMQ support"
    echo "  simtime         build with simtime support in Madara"
    echo "  help            get script usage"
    echo ""
    echo "The following environment variables are used"
    echo "  CAPNP_ROOT          - location of Cap'n Proto"
    echo "  CORES               - number of build jobs to launch with make, optional"
    echo "  MPC_ROOT            - location of MakefileProjectCreator"
    echo "  MADARA_ROOT         - location of local copy of MADARA git repository from"
    echo "                        git://git.code.sf.net/p/madara/code"
    echo "  GAMS_ROOT           - location of this GAMS git repository"
    echo "  VREP_ROOT           - location of VREP installation"
    echo "  JAVA_HOME           - location of JDK"
    echo "  LZ4_ROOT            - location of LZ4"
    echo "  ZMQ_ROOT            - location of ZeroMQ"
    echo "  SSL_ROOT            - location of OpenSSL"
    echo "  ROS_ROOT            - location of ROS (usually set by ROS installer)"
    echo "  DMPL_ROOT           - location of DART DMPL directory"
    exit
  fi
done

if [ -z $MPC_ROOT ] ; then
  export MPC_ROOT=$INSTALL_DIR/MPC
fi

if [ -z $EIGEN_ROOT ] ; then
  export EIGEN_ROOT=$INSTALL_DIR/eigen
fi

if [ -z $CAPNP_ROOT ] ; then
  export CAPNP_ROOT=$INSTALL_DIR/capnproto
fi

if [ -z $LZ4_ROOT ] ; then
  export LZ4_ROOT=$INSTALL_DIR/lz4
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

echo "JAVA has been set to $JAVA"
if [ $JAVA -eq 1 ]; then
  echo "JAVA_HOME is referencing $JAVA_HOME"
fi

echo "VREP has been set to $VREP"
if [ $VREP -eq 1 ]; then
  echo "VREP_ROOT is referencing $VREP_ROOT"
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
    export NDK_VERSION=r16b
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

fi

if [ $DOCS -eq 1 ]; then
  echo "DOCS is set to $DOCS"
fi

echo ""

unzip_strip() (
  local zip=$1
  local dest=${2:-.}
  local temp=$(mktemp -d) && unzip -d "$temp" "$zip" && mkdir -p "$dest" &&
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
    sudo apt-get install -y -f clang-5.0 libc++-dev libc++abi-dev
  fi

  if [ $JAVA -eq 1 ]; then
    sudo apt-get install -y -f oracle-java8-set-default
    sudo apt-get install -y maven
    export JAVA_HOME=/usr/lib/jvm/java-8-oracle
    rc_str="export JAVA_HOME=$JAVA_HOME"
    append_if_needed "$rc_str" "$HOME/.bashrc"
  fi

  if [ $PYTHON -eq 1 ]; then
    sudo apt-get install -y -f python2.7 python-pip
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
       git clone "git@github.com:amsurana/Boost-for-Android.git" $BOOST_ANDROID_ROOT
       cd $BOOST_ANDROID_ROOT;
       echo "Boost is cloned in $BOOST_ANDROID_ROOT"
       ./build-android.sh --boost=1.65.1 --arch=$ANDROID_ARCH $NDK_ROOT
     fi
     
     if [ ! -d $BOOST_ANDROID_ROOT ] || [ ! -d "$BOOST_ANDROID_ROOT/build/$ANDROID_ARCH" ]; then
       echo "Unable to download or setup boos. Refer README-ANDROID.md for manual setup"
       exit 1;
     fi 
  fi

  if [ $ROS -eq 1 ]; then
    sudo apt-get install -y ros-kinetic-desktop-full python-rosinstall ros-kinetic-ros-type-introspection ros-kinetic-move-base-msgs ros-kinetic-navigation libactionlib-dev libactionlib-msgs-dev libmove-base-msgs-dev

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

fi
if [ $MAC -eq 1 ]; then
  # Install boost for mac
  if [ $PREREQS -eq 1 ]; then
    brew install boost@1.59
    if [ $JAVA -eq 1 ]; then
      brew install maven
    fi
  fi
  export BOOST_ROOT=/usr/local/opt/boost@1.59/include
  export BOOST_ROOT_LIB=/usr/local/opt/boost@1.59/lib
fi


if [ $LZ4 -eq 1 ] ; then
  if [ ! -d $LZ4_ROOT  ]; then
    echo "git clone https://github.com/lz4/lz4.git $LZ4_ROOT"
    git clone https://github.com/lz4/lz4.git $LZ4_ROOT
  else
    echo "UPDATING LZ4"
    cd $LZ4_ROOT
    git pull
  fi
  LZ4_REPO_RESULT=$?

  if [ -f $LZ4_ROOT/lib/lz4.c ] ; then
    echo "mv $LZ4_ROOT/lib/lz4.c $LZ4_ROOT/lib/lz4.cpp"
    mv $LZ4_ROOT/lib/lz4.c $LZ4_ROOT/lib/lz4.cpp
  fi
fi


# check if MPC is a prereq for later packages

if [ $DMPL -eq 1 ] || [ $GAMS -eq 1 ] || [ $MADARA -eq 1 ]; then
  MPC_DEPENDENCY_ENABLED=1
fi

if [ $MPC_DEPENDENCY_ENABLED -eq 1 ] && [ ! -d $MPC_ROOT ]; then
  MPC_AS_A_PREREQ=1
fi

if [ $MADARA -eq 1 ]  && [ $PREREQS -eq 1 ]; then
  CAPNP_AS_A_PREREQ=1
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
    echo "git clone --single-branch --branch 3.3.4 --depth 1 https://github.com/eigenteam/eigen-git-mirror.git $EIGEN_ROOT"
    git clone --single-branch --branch 3.3.4 --depth 1 https://github.com/eigenteam/eigen-git-mirror.git $EIGEN_ROOT
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

if [ $CAPNP_AS_A_PREREQ -eq 1 ]; then

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
    git clone --single-branch --branch release-0.6.1 --depth 1 https://github.com/capnproto/capnproto.git $CAPNP_ROOT
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
    export CC=${CROSS_PATH}-gcc
    export CXX=${CROSS_PATH}-clang++
    export LD=${CROSS_PATH}-ld
    export RANLIB=${CROSS_PATH}-ranlib
    export STRIP=${CROSS_PATH}-strip

    ./configure --host=${ANDROID_TOOLCHAIN} --with-sysroot=${SYSROOT} CPPFLAGS="${CPPLAGS} --sysroot=${SYSROOT} -I${SYSROOT}/usr/include -I${NDK_TOOLS}/include -std=c++11" LDFLAGS="${LDFLAGS} -L${SYSROOT}/usr/lib -L${NDK_TOOLS}/lib -L${NDK_TOOLS}/${ANDROID_TOOLCHAIN}/lib" LIBS='-lc++_shared'

  else 
    export PATH="$CAPNP_ROOT/c++:$PATH"
    export LD_LIBRARY_PATH="$CAPNP_ROOT/c++/.libs:$LD_LIBRARY_PATH"
    if [ $CLANG -ne 0 ] && [ $MAC -eq 0 ]; then
      export CC=clang-5.0
      export CXX=clang++-5.0
      export CXXFLAGS="-stdlib=libc++ -I/usr/include/libcxxabi"
    fi

   ./configure

  fi

   make -j$CORES capnp capnpc-c++ libcapnp-json.la 
   CAPNP_BUILD_RESULT=$?

   if [ ! -d $CAPNP_ROOT/c++/.libs ]; then
       echo "CapNProto is not built properly."
       exit 1;
   fi
 #Prereq if ends
else
  echo "NOT CHECKING CAPNPROTO"
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
  if [ -z $SSL_ROOT ]; then
    if [ $MAC -eq 0 ]; then
      export SSL_ROOT=/usr
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

if [ $DMPL -eq 1 ] || [ $GAMS -eq 1 ] ; then
  MADARA_DEPENDENCY_ENABLED=1
fi

# check if MADARA is a prereq for later packages
if [ $MADARA_DEPENDENCY_ENABLED -eq 1 ] && [ ! -d $MADARA_ROOT ]; then
  MADARA_AS_A_PREREQ=1
fi

if [ $MADARA -eq 1 ] || [ $MADARA_AS_A_PREREQ -eq 1 ]; then

  echo "LD_LIBRARY_PATH for MADARA compile is $LD_LIBRARY_PATH"

  if ! ls $CAPNP_ROOT/c++/.libs/libcapnp-json* > /dev/null 2>&1; then 
   echo "Cap'Nproto is not built properly or not installed. Please run base_build with 'prereqs' option or check out troubleshooting steps in GAMS Installation Wiki."; 
exit 1;
fi
  

  cd $INSTALL_DIR

  if [ ! -d $MADARA_ROOT ] ; then
    echo "DOWNLOADING MADARA"
    echo "git clone -b master --depth 1 https://github.com/jredmondson/madara.git $MADARA_ROOT"
    git clone -b master --depth 1 https://github.com/jredmondson/madara.git $MADARA_ROOT
    MADARA_REPO_RESULT=$?
  else
    echo "UPDATING MADARA"
    cd $MADARA_ROOT
    git pull
    MADARA_REPO_RESULT=$?
    echo "CLEANING MADARA OBJECTS"
    if [ $CLEAN -eq 1 ] ; then
      make realclean -j $CORES
      rm GNUmakefile*
    fi

  fi
  cd $MADARA_ROOT
  echo "GENERATING MADARA PROJECT"
  echo "perl $MPC_ROOT/mwc.pl -type make -features android=$ANDROID,python=$PYTHON,java=$JAVA,tests=$TESTS,docs=$DOCS,ssl=$SSL,zmq=$ZMQ,simtime=$SIMTIME,clang=$CLANG,debug=$DEBUG MADARA.mwc"
  perl $MPC_ROOT/mwc.pl -type make -features lz4=$LZ4,android=$ANDROID,python=$PYTHON,java=$JAVA,tests=$TESTS,docs=$DOCS,ssl=$SSL,zmq=$ZMQ,simtime=$SIMTIME,clang=$CLANG,debug=$DEBUG MADARA.mwc

  if [ $JAVA -eq 1 ]; then
    echo "DELETING MADARA JAVA CLASSES"
    # sometimes the jar'ing will occur before all classes are actually built when performing
    # multi-job builds, fix by deleting class files and recompiling with single build job
    find . -name "*.class" -delete
  fi

  echo "BUILDING MADARA"
  echo "make depend android=$ANDROID java=$JAVA tests=$TESTS docs=$DOCS ssl=$SSL zmq=$ZMQ simtime=$SIMTIME python=$PYTHON -j $CORES"
  make depend lz4=$LZ4 android=$ANDROID java=$JAVA tests=$TESTS docs=$DOCS ssl=$SSL zmq=$ZMQ simtime=$SIMTIME python=$PYTHON -j $CORES
  echo "make android=$ANDROID java=$JAVA tests=$TESTS docs=$DOCS ssl=$SSL zmq=$ZMQ simtime=$SIMTIME python=$PYTHON -j $CORES"
  make lz4=$LZ4 android=$ANDROID java=$JAVA tests=$TESTS docs=$DOCS ssl=$SSL zmq=$ZMQ simtime=$SIMTIME python=$PYTHON -j $CORES
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
    $GAMS_ROOT/scripts/simulation/remoteApiConnectionsGen.pl 19905 20


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

# if gams has been specified, or if dmpl is specified and GAMS_ROOT doesn't exist
if [ $GAMS -eq 1 ] || [ $GAMS_AS_A_PREREQ -eq 1 ]; then

  # build GAMS
  if [ -z $GAMS_ROOT ] ; then
    export GAMS_ROOT=$INSTALL_DIR/gams
    echo "SETTING GAMS_ROOT to $GAMS_ROOT"
  fi
  if [ ! -d $GAMS_ROOT ] ; then
    echo "DOWNLOADING GAMS"
    echo "git clone -b master --depth 1 --single-branch https://github.com/jredmondson/gams.git $GAMS_ROOT"
    git clone -b master --depth 1 --single-branch https://github.com/jredmondson/gams.git $GAMS_ROOT
    GAMS_REPO_RESULT=$?
    
  else
    echo "UPDATING GAMS"
    cd $GAMS_ROOT
    git pull
    GAMS_REPO_RESULT=$?

    echo "CLEANING GAMS OBJECTS"
    if [ $CLEAN -eq 1 ] ; then
      make realclean -j $CORES
      rm GNUmakefile*
    fi

  fi
    
  cd $GAMS_ROOT

  echo "GENERATING GAMS PROJECT"
  echo "perl $MPC_ROOT/mwc.pl -type make -features java=$JAVA,ros=$ROS,vrep=$VREP,tests=$TESTS,android=$ANDROID,docs=$DOCS,clang=$CLANG,simtime=$SIMTIME,debug=$DEBUG gams.mwc"
  perl $MPC_ROOT/mwc.pl -type make -features java=$JAVA,ros=$ROS,vrep=$VREP,tests=$TESTS,android=$ANDROID,docs=$DOCS,clang=$CLANG,simtime=$SIMTIME,debug=$DEBUG gams.mwc

  if [ $JAVA -eq 1 ]; then
    # sometimes the jar'ing will occur before all classes are actually built when performing
    # multi-job builds, fix by deleting class files and recompiling with single build job
    find . -name "*.class" -delete
  fi

  echo "BUILDING GAMS"
  echo "make depend java=$JAVA ros=$ROS vrep=$VREP tests=$TESTS android=$ANDROID simtime=$SIMTIME docs=$DOCS -j $CORES"
  make depend java=$JAVA ros=$ROS vrep=$VREP tests=$TESTS android=$ANDROID simtime=$SIMTIME docs=$DOCS -j $CORES
  echo "make java=$JAVA ros=$ROS vrep=$VREP tests=$TESTS android=$ANDROID simtime=$SIMTIME docs=$DOCS -j $CORES"
  make java=$JAVA ros=$ROS vrep=$VREP tests=$TESTS android=$ANDROID simtime=$SIMTIME docs=$DOCS -j $CORES
  GAMS_BUILD_RESULT=$?
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
 if [ $ANDROID -eq 1 ]; then
    echo "Building Demo Android app"
    cd $GAMS_ROOT/port/android
    chmod a+x build.sh
    ./build.sh
   cd $GAMS_ROOT
  fi
else
  echo "NOT BUILDING GAMS"
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
    echo "DOWNLOADING GAMS"
    git clone --depth 1 -b release-0.4.0 https://github.com/cps-sei/dmplc.git $DMPL_ROOT
    DART_REPO_RESULT=$?
    
  else
    echo "UPDATING DMPL"
    cd $DMPL_ROOT
    git pull

    echo "CLEANING GAMS OBJECTS"
    if [ $CLEAN -eq 1]; then
      make clean -j $CORES
    fi

  fi

  cd $DMPL_ROOT
  make depend MZSRM=0 -j $CORES
  make MZSRM=0 -j $CORES
  DART_BUILD_RESULT=$?
fi

if [ $VREP_CONFIG -eq 1 ]; then
  echo "CONFIGURING 20 VREP PORTS"
  $GAMS_ROOT/scripts/simulation/remoteApiConnectionsGen.pl 19905 20
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
  if [ $CAPNP_AS_A_PREREQ -eq 1 ]; then
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
  fi
  echo "  MADARA"
  if [ $MADARA_REPO_RESULT -eq 0 ]; then
    echo -e "    REPO=\e[92mPASS\e[39m"
  else
    echo -e "    REPO=\e[91mFAIL\e[39m"
    (( BUILD_ERRORS++ ))
  fi
  if [ $MADARA_BUILD_RESULT -eq 0 ]; then
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
  if [ $GAMS_REPO_RESULT -eq 0 ]; then
    echo -e "    REPO=\e[92mPASS\e[39m"
  else
    echo -e "    REPO=\e[91mFAIL\e[39m"
    (( BUILD_ERRORS++ ))
  fi
  if [ $GAMS_BUILD_RESULT -eq 0 ]; then
    echo -e "    BUILD=\e[92mPASS\e[39m"
  else
    echo -e "    BUILD=\e[91mFAIL\e[39m"
    (( BUILD_ERRORS++ ))
  fi
fi

echo -e ""
echo -e "Make sure to update your environment variables to the following"
echo -e "\e[96mexport MPC_ROOT=$MPC_ROOT"
echo -e "export EIGEN_ROOT=$EIGEN_ROOT"
echo -e "export CAPNP_ROOT=$CAPNP_ROOT"
echo -e "export MADARA_ROOT=$MADARA_ROOT"
echo -e "export GAMS_ROOT=$GAMS_ROOT"
echo -e "export VREP_ROOT=$VREP_ROOT"

if [ $SSL -eq 1 ]; then
  if [ -z $SSL_ROOT ]; then
    export SSL_ROOT=/usr
  fi
  echo -e "export SSL_ROOT=$SSL_ROOT"
fi

if [ $LZ4 -eq 1 ]; then
  echo -e "export LZ4_ROOT=$LZ4_ROOT"
fi

if [ $ZMQ -eq 1 ]; then
  echo -e "export ZMQ_ROOT=$ZMQ_ROOT"
fi

if [ $JAVA -eq 1 ]; then
  echo -e "export JAVA_HOME=$JAVA_HOME"
fi

if [ $DMPL -eq 1 ]; then
  echo -e "export DMPL_ROOT=$DMPL_ROOT"
fi

if [ $ANDROID -eq 1 ]; then
  echo -e "export NDK_ROOT=$NDK_ROOT"
  echo -e "export NDK_TOOLS=$NDK_TOOLS"
  echo -e "export SYSROOT=$SYSROOT"
  echo -e "export ANDROID_ARCH=$ANDROID_ARCH"
  echo -e "export BOOST_ANDROID_ROOT=$BOOST_ANDROID_ROOT"
fi

if [ $MAC -eq 0 ]; then
  echo -e "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$MADARA_ROOT/lib:\$GAMS_ROOT/lib:\$VREP_ROOT:\$CAPNP_ROOT/c++/.libs"
else
  echo -e "export DYLD_LIBRARY_PATH=\$DYLD_LIBRARY_PATH:\$MADARA_ROOT/lib:\$GAMS_ROOT/lib:\$VREP_ROOT:\$CAPNP_ROOT/c++/.libs"
fi
echo -e "export PATH=\$PATH:\$MPC_ROOT:\$VREP_ROOT:\$CAPNP_ROOT/c++"

if [ $DMPL -eq 1 ]; then
  echo -e "export PATH=\$PATH:\$DMPL_ROOT/src/DMPL:\$DMPL_ROOT/src/vrep"
fi

echo -e "\e[39m"
echo -e "IF YOUR BUILD IS NOT COMPILING, MAKE SURE THE ABOVE VARIABLES ARE SET"
echo -e "IN YOUR BASHRC OR TERMINAL."
echo -e ""

echo "BUILD_ERRORS=$BUILD_ERRORS"
exit $BUILD_ERRORS




