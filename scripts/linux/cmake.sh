#!/bin/bash

SCRIPT_PATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

PROJECT_PATH=$SCRIPT_PATH/../..

echo SCRIPT_PATH=$SCRIPT_PATH
echo PROJECT_PATH=$PROJECT_PATH

cd $PROJECT_PATH
mkdir build
mkdir install
cd build

cmake -D'CMAKE_INSTALL_PREFIX=/usr/local' -DCMAKE_PREFIX_PATH=$PROJECT_PATH/install ..
cmake --build . --config debug -j$(nproc)
cmake --build . --config release
sudo cmake --build . --target install --config release
sudo cmake --build . --target install --config debug
sudo ldconfig
