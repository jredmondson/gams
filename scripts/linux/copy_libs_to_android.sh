#!/bin/bash

#my_dir="$(dirname "$0")"
#source "$my_dir/env_vars.sh"

#List the devices on the screen for your viewing pleasure
adb devices
echo

#Find USB devices only (no emulators, genymotion or connected devices
declare -a deviceArray=(`adb devices -l | grep -v emulator | grep -v vbox | grep " device " | awk '{print $1}'`)

echo "found ${#deviceArray[@]} device(s)"
echo

for index in ${!deviceArray[*]}
do
  adb -s ${deviceArray[index]} push $NDK/arm-linux-androideabi/lib/libgnustl_shared.so /data/local/tmp/libgnustl_shared.so
  adb -s ${deviceArray[index]} push $ACE_ROOT/ace/libACE.so /data/local/tmp/libACE.so
  adb -s ${deviceArray[index]} push $MADARA_ROOT/libMADARA.so /data/local/tmp/libMADARA.so
  adb -s ${deviceArray[index]} push $MADARA_ROOT/network_counter_filter /data/local/tmp/network_counter_filter
  adb -s ${deviceArray[index]} push $MADARA_ROOT/lib/madara.dex /data/local/tmp/madara.dex
done
