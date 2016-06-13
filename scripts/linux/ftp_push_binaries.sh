#!/bin/bash
# copies files from localhost to drone at IP
#
# build_ar_drone.sh will build the necessary files and place them in the 
# location that this script expects

# bring in other variables
source $GAMS_ROOT/scripts/linux/common.sh

# copy files for gdb to use
rm -rf lib
cp -r $DRONE_DIR lib

# strip binaries
echo "Strip binaries"
${ARM_PREFIX}strip $DRONE_DIR/*

# setup ip address
IP=192.168.1.1
echo "IP start as $IP"

if [ ! -z $1 ]; then
    IP=$1
    echo "FTP IP is set to $IP"
fi

# copy scripts file
cd $GAMS_ROOT/scripts/linux
ftp -n -v $IP << END_SCRIPTS_FTP

binary

delete swarmSetup.sh
delete hosts

put swarmSetup.sh
put hosts

quit

END_SCRIPTS_FTP

# mark scripts as executable
telnet $IP << END_SCRIPTS_TELNET

cd data/video
chmod a+x swarmSetup.sh 
cp hosts /etc
exit

END_SCRIPTS_TELNET

# copy madara to drone
cd $DRONE_DIR
ftp -n -v $IP << END_MADARA_FTP

binary

delete libMADARA.so 
delete network_profiler 
delete test_file_rebroadcasts 
delete test_fragmentation 
delete test_rebroadcast_ring
delete test_reasoning_throughput 
delete test_udp 
delete test_broadcast 
delete profile_architecture
delete madara_version

put libMADARA.so 
put network_profiler 
put test_file_rebroadcasts 
put test_fragmentation 
put test_rebroadcast_ring
put test_reasoning_throughput 
put test_udp 
put test_broadcast 
put profile_architecture
put madara_version

quit

END_MADARA_FTP

# set madara files as executable
telnet $IP << END_MADARA_TELNET

cd /data/video
chmod +x network_profiler 
chmod +x test_file_rebroadcasts 
chmod +x test_fragmentation 
chmod +x test_rebroadcast_ring
chmod +x test_reasoning_throughput 
chmod +x test_udp 
chmod +x test_broadcast 
chmod +x profile_architecture
chmod +x madara_version
exit

END_MADARA_TELNET

# copy ace to drone
ftp -n -v $IP << END_ACE_FTP

binary
delete libACE.so
put libACE.so
quit

END_ACE_FTP

# copy gams to drone
ftp -n -v $IP << END_GAMS_FTP

binary
delete libGAMS.so
delete gams_controller
put libGAMS.so
put gams_controller
quit

END_GAMS_FTP

# set gams files as executable
telnet $IP << END_GAMS_TELNET

cd /data/video
chmod +x gams_controller
exit

END_GAMS_TELNET

# copy drk to drone
ftp -n -v $IP << END_DRK_FTP

binary
delete libdrk.so
delete all_sensor_data
delete simple_flight
put libdrk.so
put all_sensor_data
put simple_flight
quit

END_DRK_FTP

# set drk files as executable
telnet $IP << END_DRK_TELNET

cd /data/video
chmod +x all_sensor_data
chmod +x simple_flight
exit

END_DRK_TELNET

# copy testing scripts
cd $GAMS_ROOT/scripts/drone_rk/testing/takeoff_land
ftp -n -v $IP << END_SCRIPTS_FTP

binary
delete madara_init.mf
delete test.sh
put madara_init.mf
put test.sh
quit

END_SCRIPTS_FTP

# mark as executable
telnet $IP << END_SCRIPTS_TELNET

cd /data/video
chmod +x test.sh
exit

END_SCRIPTS_TELNET

exit 0
