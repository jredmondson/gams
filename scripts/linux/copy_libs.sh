#!/bin/bash

#my_dir="$(dirname "$0")"
#source "$my_dir/env_vars.sh"

declare -a APPS=("message_profiling" "GAMSController")

for app in "${APPS[@]}"
do
  LIB_DIR=$GAMS_ROOT/src/android/$app/app/libs
  rm -rf $LIB_DIR
  mkdir -p $LIB_DIR/armeabi
  cp $MADARA_ROOT/libMADARA.so $LIB_DIR/armeabi
  cp $MADARA_ROOT/lib/madara.jar $LIB_DIR
  cp $GAMS_ROOT/libGAMS.so $LIB_DIR/armeabi
  cp $GAMS_ROOT/lib/gams.jar $LIB_DIR
  cp $NDK/arm-linux-androideabi/lib/libgnustl_shared.so $LIB_DIR/armeabi

  mkdir -p $LIB_DIR/../src/main/assets/areas
  cp $GAMS_ROOT/scripts/simulation/areas/cmu.mf $LIB_DIR/../src/main/assets/areas/cmu.mf
  cp $GAMS_ROOT/scripts/simulation/areas/small.mf $LIB_DIR/../src/main/assets/areas/small.mf
  cp $GAMS_ROOT/scripts/simulation/madara_init_common.mf $LIB_DIR/../src/main/assets/madara_init_common.mf
  cp $GAMS_ROOT/resources/vrep/Quadricopter_NoCamera.ttm $LIB_DIR/../src/main/assets/Quadricopter_NoCamera.ttm
done
