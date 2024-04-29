#!/bin/bash
#
# Copyright (c) 2024 James Edmondson and Koshee LLC All Rights Reserved.
#

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PROJECT_DIR=$SCRIPT_DIR

pushd $(pwd)

cd $PROJECT_DIR
if [ ! -d $PROJECT_DIR/build ]; then
  mkdir build
fi

cd build
cmake ..
cmake --build . --config Release -- -j$(nproc)

popd
