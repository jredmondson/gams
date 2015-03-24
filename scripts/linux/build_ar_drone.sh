#!/bin/bash
# Build the required libraries for the ARM target Parrot AR.Drone 2.0
#
# There are several expected environment variables
#   $ARM_PREFIX   - full path prefix of arm compiler
#                   ex. ~/arm-none-linux-gnueabi-
#                   requires that ${ARM_PREFIX}g++ and ${ARM_PREFIX}strip are
#                   available
#   $CORES        - number of build jobs to launch with make
#   $ACE_ROOT     - location of local copy of ACE subversion repository from
#                   svn://svn.dre.vanderbilt.edu/DOC/Middleware/sets-anon/ACE
#                   Revision 97777 from Jun 6, 2014 is known to work
#   $MADARA_ROOT  - location of local copy of MADARA git repository from
#                   http://madara.googlecode.com/svn/trunk/
#   $GAMS_ROOT    - location of this GAMS git repository

# quit on any failure
set -e

source $GAMS_ROOT/scripts/linux/common.sh

# echo build information
echo "Using ARM prefix of $ARM_PREFIX with $CORES build jobs"
echo "MADARA will be built from $MADARA_ROOT"
echo "ACE will be built from $ACE_ROOT"
echo "GAMS will be built from $GAMS_ROOT"
echo "Drone files will be copied to $DRONE_DIR"
echo ""

# prepare destination directory
cd $GAMS_ROOT
rm -rf $DRONE_DIR
mkdir $DRONE_DIR

# building ACE
echo "Configuring ACE for ARM"

echo "#include \"ace/config-linux.h\"" > $ACE_ROOT/ace/config.h
echo "CROSS_COMPILE=$ARM_PREFIX" > $ACE_ROOT/include/makeinclude/platform_macros.GNU
echo "no_hidden_visibility=1" >> $ACE_ROOT/include/makeinclude/platform_macros.GNU
echo versioned_so=0 >> $ACE_ROOT/include/makeinclude/platform_macros.GNU
echo OCFLAGS=-Os >> $ACE_ROOT/include/makeinclude/platform_macros.GNU
echo cross_compile=1 >> $ACE_ROOT/include/makeinclude/platform_macros.GNU
echo ARM=1 >> $ACE_ROOT/include/makeinclude/platform_macros.GNU
echo "include \$(ACE_ROOT)/include/makeinclude/platform_linux.GNU" >> $ACE_ROOT/include/makeinclude/platform_macros.GNU

echo "Building ACE"
cd $ACE_ROOT/ace
perl $ACE_ROOT/bin/mwc.pl -type gnuace ace.mwc
make realclean
make optimize=0 -j $CORES
cp libACE.so $DRONE_DIR

# build MADARA
echo ""
echo "Building MADARA"
cd $MADARA_ROOT
perl $ACE_ROOT/bin/mwc.pl -type gnuace MADARA.mwc
make realclean
make optimize=0 tests=1 -j $CORES
cp libMADARA.so $DRONE_DIR
cp network_profiler $DRONE_DIR
cp test_file_rebroadcasts $DRONE_DIR
cp test_fragmentation $DRONE_DIR
cp test_rebroadcast_ring $DRONE_DIR
cp test_reasoning_throughput $DRONE_DIR
cp test_udp $DRONE_DIR
cp test_broadcast $DRONE_DIR
cp profile_architecture $DRONE_DIR
cp madara_version $DRONE_DIR

# build Drone-RK
echo ""
echo "Build Drone-RK"
cd $DRK_ROOT
make libdrk.so
cp lib/libdrk.so $DRONE_DIR
make all_sensor_data
cp bin/all_sensor_data $DRONE_DIR
make simple_flight
cp bin/simple_flight $DRONE_DIR

# build GAMS
echo ""
echo "Build GAMS"
cd $GAMS_ROOT
perl $ACE_ROOT/bin/mwc.pl -type gnuace -features dronerk=1 gams.mwc 
make realclean
make optimize=0 dronerk=1 -j $CORES
cp libGAMS.so $DRONE_DIR
cp gams_controller $DRONE_DIR
