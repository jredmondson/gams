#!/bin/bash

LIBS_DIR=$GAMS_ROOT/src/android/message_profiling/app/libs

rm -rf $LIBS_DIR
mkdir -p $LIBS_DIR/armeabi
cp $ACE_ROOT/ace/libACE.so $LIBS_DIR/armeabi
cp $MADARA_ROOT/libMADARA.so $LIBS_DIR/armeabi
cp $MADARA_ROOT/lib/madara.jar $LIBS_DIR
cp $GAMS_ROOT/libGAMS.so $LIBS_DIR/armeabi
cp $GAMS_ROOT/lib/gams.jar $LIBS_DIR
cp $NDK/arm-linux-androideabi/lib/libgnustl_shared.so $LIBS_DIR/armeabi
