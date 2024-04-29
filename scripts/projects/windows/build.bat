@echo off

SET SCRIPTS_DIR=%~dp0
SET SCRIPTS_DIR=%SCRIPTS_DIR:~0,-1%

SET RUN_LOCATION=%CD%

setlocal enableDelayedExpansion

:END_OF_SCRIPT

endlocal

cd %RUN_LOCATION%
mkdir build
cd build
cmake ..
cmake --build . --config Release -- -j4

@echo on
