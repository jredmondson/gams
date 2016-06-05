#!/bin/bash
#
# Copyright (c) 2015 Carnegie Mellon University. All Rights Reserved.
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
#   $ACE_ROOT     - location of local copy of ACE subversion repository from
#                   svn://svn.dre.vanderbilt.edu/DOC/Middleware/sets-anon/ACE
#   $MADARA_ROOT  - location of local copy of MADARA git repository from
#                   git://git.code.sf.net/p/madara/code
#   $GAMS_ROOT    - location of this GAMS git repository
#   $VREP_ROOT    - location of VREP installation, if applicable
#
# For android
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
STRIP=0
ODROID=0
ACE=0
MADARA=0
GAMS=0
PREREQS=0
STRIP_EXE=strip
VREP_INSTALLER="V-REP_PRO_EDU_V3_3_0_64_Linux.tar.gz"
INSTALL_DIR=`pwd`
SCRIPTS_DIR=`dirname $0`

for var in "$@"
do
  if [ "$var" = "tests" ]; then
    TESTS=1
  elif [ "$var" = "prereqs" ]; then
    PREREQS=1
  elif [ "$var" = "vrep" ]; then
    VREP=1
  elif [ "$var" = "java" ]; then
    JAVA=1
  elif [ "$var" = "ros" ]; then
    ROS=1
  elif [ "$var" = "ace" ]; then
    ACE=1
  elif [ "$var" = "madara" ]; then
    MADARA=1
  elif [ "$var" = "gams" ]; then
    GAMS=1
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
    echo "  ace             build ACE"
    echo "  android         build android libs, turns on java"
    echo "  gams            build GAMS"
    echo "  java            build java jar"
    echo "  madara          build MADARA"
    echo "  odroid          target ODROID computing platform"
    echo "  prereqs         use apt-get to install prereqs"
    echo "  ros             build ROS platform classes"
    echo "  strip           strip symbols from the libraries"
    echo "  tests           build test executables"
    echo "  vrep            build with vrep support"
    echo "  help            get script usage"
    echo ""
    echo "The following environment variables are used"
    echo "  CORES               - number of build jobs to launch with make, optional"
    echo "  ACE_ROOT            - location of local copy of ACE subversion repository from"
    echo "                        svn://svn.dre.vanderbilt.edu/DOC/Middleware/sets-anon/ACE"
    echo "  MADARA_ROOT         - location of local copy of MADARA git repository from"
    echo "                        git://git.code.sf.net/p/madara/code"
    echo "  GAMS_ROOT           - location of this GAMS git repository"
    echo "  VREP_ROOT           - location of VREP installation"
    echo "  JAVA_HOME           - location of JDK"
    exit
  fi
done

if [ -z $ACE_ROOT ] ; then
  export ACE_ROOT=$INSTALL_DIR/ace/ACE_wrappers
fi

# echo build information
echo "INSTALL_DIR will be $INSTALL_DIR"
echo "Using $CORES build jobs"

if [ $PREREQS -eq 0 ]; then
  echo "No pre-requisites will be installed"
else
  echo "Pre-requisites will be installed"
fi

echo "ACE_ROOT is set to $ACE_ROOT"
if [ $ACE -eq 0 ]; then
  echo "ACE will not be built"
else
  echo "ACE will be built"
fi

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
fi

echo ""

if [ $PREREQS -eq 1 ]; then
  sudo apt-get install -f build-essential subversion git-core perl
  
  if [ $JAVA -eq 1 ]; then
    sudo add-apt-repository ppa:webupd8team/java
    sudo apt-get update
    sudo apt-get install -f oracle-java8-set-default
    echo "export JAVA_HOME=/usr/lib/jvm/java-8-oracle" >> $HOME/.bashrc
  fi
fi

if [ $ACE -eq 1 ]; then

  # build ACE, all build information (compiler and options) will be set here
  if [ ! -d $ACE_ROOT ] ; then
    echo "DOWNLOADING ACE"
    svn checkout --quiet svn://svn.dre.vanderbilt.edu/DOC/Middleware/sets-anon/ACE $ACE_ROOT/../../ace
    echo "CONFIGURING ACE"
    if [ $ANDROID -eq 1 ]; then
      # use the android specific files, we use custom config file for android due to build bug in ACE
      echo "#include \"$GAMS_ROOT/scripts/linux/config-android.h\"" > $ACE_ROOT/ace/config.h

      # Android does not support versioned libraries and requires cross-compiling
      echo -e "no_hidden_visibility=1\nversioned_so=0\nCROSS_COMPILE=$LOCAL_CROSS_PREFIX\ninclude \$(ACE_ROOT)/include/makeinclude/platform_android.GNU" > $ACE_ROOT/include/makeinclude/platform_macros.GNU
    else
      # use linux defaults
      echo "#include \"ace/config-linux.h\"" > $ACE_ROOT/ace/config.h
      echo -e "no_hidden_visibility=1\ninclude \$(ACE_ROOT)/include/makeinclude/platform_linux.GNU" > $ACE_ROOT/include/makeinclude/platform_macros.GNU
    fi
  fi
  
  echo "ENTERING $ACE_ROOT"
  cd $ACE_ROOT/ace
  echo "GENERATING ACE PROJECT"
  perl $ACE_ROOT/bin/mwc.pl -type gnuace ace.mwc
  echo "CLEANING ACE OBJECTS"
  make realclean -j $CORES
  echo "BUILDING ACE"
  make -j $CORES
  if [ $STRIP -eq 1 ]; then
    echo "STRIPPING ACE"
    $STRIP_EXE libACE.so*
  fi
else
  echo "NOT BUILDING ACE"
fi

if [ $MADARA -eq 1 ]; then
  # build MADARA
  if [ -z $MADARA_ROOT ] ; then
    export MADARA_ROOT=$INSTALL_DIR/madara
    echo "SETTING MADARA_ROOT to $MADARA_ROOT"
  fi
  if [ ! -d $MADARA_ROOT ] ; then
    echo "DOWNLOADING MADARA"
    git clone http://git.code.sf.net/p/madara/code $MADARA_ROOT
  else
    echo "UPDATING MADARA"
    cd $MADARA_ROOT
    git pull
    echo "CLEANING MADARA OBJECTS"
    make realclean -j $CORES

  fi
  cd $MADARA_ROOT
  echo "GENERATING MADARA PROJECT"
  perl $ACE_ROOT/bin/mwc.pl -type gnuace -features android=$ANDROID,java=$JAVA,tests=$TESTS MADARA.mwc

  if [ $JAVA -eq 1 ]; then
    echo "DELETING MADARA JAVA CLASSES"
    # sometimes the jar'ing will occur before all classes are actually built when performing
    # multi-job builds, fix by deleting class files and recompiling with single build job
    find . -name "*.class" -delete
  fi

  echo "BUILDING MADARA"
  make android=$ANDROID java=$JAVA tests=$TESTS -j $CORES

  if [ $STRIP -eq 1 ]; then
    echo "STRIPPING MADARA"
    $STRIP_EXE libMADARA.so*
  fi
else
  echo "NOT BUILDING MADARA"
fi

if [ $GAMS -eq 1 ]; then

  # build GAMS
  if [ -z $GAMS_ROOT ] ; then
    export GAMS_ROOT=$INSTALL_DIR/gams
    echo "SETTING GAMS_ROOT to $GAMS_ROOT"
  fi
  if [ ! -d $GAMS_ROOT ] ; then
    echo "DOWNLOADING GAMS"
    git clone -b master --single-branch https://github.com/jredmondson/gams.git $GAMS_ROOT
    
  else
    echo "UPDATING GAMS"
    cd $GAMS_ROOT
    git pull

    echo "CLEANING GAMS OBJECTS"
    make realclean -j $CORES

  fi

  if [ $VREP -eq 1 ]; then
    if [ ! $VREP_ROOT ] ; then
      export VREP_ROOT=$INSTALL_DIR/vrep
      echo "SETTING VREP_ROOT to $VREP_ROOT"
    fi
    if [ ! -d $VREP_ROOT ]; then 
      cd $INSTALL_DIR
      echo "DOWNLOADING VREP"
      wget http://coppeliarobotics.com/$VREP_INSTALLER
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

    
  cd $GAMS_ROOT

  echo "GENERATING GAMS PROJECT"
  perl $ACE_ROOT/bin/mwc.pl -type gnuace -features java=$JAVA,ros=$ROS,vrep=$VREP,tests=$TESTS,android=$ANDROID gams.mwc

  if [ $JAVA -eq 1 ]; then
    # sometimes the jar'ing will occur before all classes are actually built when performing
    # multi-job builds, fix by deleting class files and recompiling with single build job
    find . -name "*.class" -delete
  fi

  echo "BUILDING GAMS"
  make java=$JAVA ros=$ROS vrep=$VREP tests=$TESTS android=$ANDROID -j $CORES

  if [ $STRIP -eq 1 ]; then
    echo "STRIPPING GAMS"
    $STRIP_EXE libGAMS.so*
  fi
else
  echo "NOT BUILDING GAMS"
fi
  
echo "BUILD COMPLETE"
echo "Make sure to update your environment variables to the following"
echo "export ACE_ROOT=$ACE_ROOT"
echo "export MADARA_ROOT=$MADARA_ROOT"
echo "export GAMS_ROOT=$GAMS_ROOT"
echo "export VREP_ROOT=$VREP_ROOT"

