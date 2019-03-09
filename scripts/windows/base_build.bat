@echo off

set RUN_LOCATION=%CD%

setlocal enableDelayedExpansion

SET VREP=0
SET VREP_CONFIG=0
SET JAVA=0
SET ACE=0
SET CLEAN=1
SET GAMS=0
SET MADARA=0
SET TESTS=0
SET TUTORIALS=0
SET DOCS=0
SET VS_VERSION="vs2017"
SET PREREQS=0

SET CAPNP_REPO_RESULT=0
SET CAPNP_BUILD_RESULT=0

SET FORCE_BOOST=0
SET FORCE_CAPNP=0
SET FORCE_OSC=0

ECHO Parsing arguments

FOR %%x in (%*) do (
   
   IF "%%x" == "docs" (
     echo Build will enable doxygen documentation
     SET DOCS=1
   ) ELSE IF "%%x" == "forceboost" (
     echo Prereqs will force Boost install
     SET FORCE_BOOST=1
     SET PREREQS=1
   ) ELSE IF "%%x" == "forcecapnp" (
     echo Prereqs will force Capnp
     SET FORCE_CAPNP=1
     SET PREREQS=1
   ) ELSE IF "%%x" == "forceosc" (
     echo Prereqs will force OSC
     SET FORCE_OSC=1
     SET PREREQS=1
   ) ELSE IF "%%x" == "gams" (
     echo Build will enable GAMS
     SET GAMS=1
   ) ELSE IF "%%x" == "java" (
     echo Build will enable Java support
     SET JAVA=1
   ) ELSE IF "%%x" == "madara" (
     echo Build will enable MADARA
     SET MADARA=1
   ) ELSE IF "%%x" == "noclean" (
     echo Build will try to minimize rebuilds 
     SET CLEAN=0
   ) ELSE IF "%%x" == "prereqs" (
     echo Build will enable prerequisites
     SET PREREQS=1
   ) ELSE IF "%%x" == "tests" (
     echo Build will enable tests
     SET TESTS=1
   ) ELSE IF "%%x" == "tutorials" (
     echo Build will enable tutorials
     SET TUTORIALS=1
   ) ELSE IF "%%x" == "vrep" (
     echo Build will enable vrep
     SET VREP=1
   ) ELSE IF "%%x" == "vrep_config" (
     echo Build will configure vrep
     SET VREP_CONFIG=1
   ) ELSE IF "%%x" == "vrep-config" (
     echo Build will configure vrep
     SET VREP_CONFIG=1
   ) ELSE IF "%%x" == "vc12" (
     echo Build will enable vrep
     SET VS_VERSION="vc12"
   ) ELSE IF "%%x" == "vc14" (
     echo Build will enable vrep
     SET VS_VERSION="vc14"
   ) ELSE IF "%%x" == "vs2017" (
     echo Build will enable vrep
     SET VS_VERSION="vs2017"
   ) ELSE (
     echo ERROR: Bad argument "%%x"
     echo   Appropriate arguments are any combination of 
     echo     docs        Enable doxygen documentation generation
     echo     gams        Build GAMS
     echo     java        Enable Java support
     echo     madara      Build MADARA
     echo     prereqs     Build prereqs
     echo     tests       Build tests
     echo     tutorials   Build tutorials
     echo     vrep        Enable VREP support
     echo     vrep-config Configure VREP installation for 20 agents
     echo     vc12        Generate Visual Studio 2012 solutions
     echo     vc14        Generate Visual Studio 2014 solutions
     echo     vs2017      Generate Visual Studio 2017 solutions

     GOTO  END_OF_SCRIPT
   )
)

echo Building options are:
echo   docs=%DOCS%
echo   gams=%GAMS%
echo   madara=%MADARA%
echo   prereqs=%PREREQS%
echo   tests=%TESTS%
echo   tutorials=%TUTORIALS%
echo   vrep=%VREP%

:: Create environment variables if they don't exist
if [%BOOST_ROOT%] == [] (
  SET BOOST_ROOT=%RUN_LOCATION%\boost
)
if [%CAPNP_ROOT%] == [] (
  SET CAPNP_ROOT=%RUN_LOCATION%\capnproto
)
if [%EIGEN_ROOT%] == [] (
  SET EIGEN_ROOT=%RUN_LOCATION%\eigen
)
if [%GAMS_ROOT%] == [] (
  SET GAMS_ROOT=%RUN_LOCATION%\gams
)
if [%LZ4_ROOT%] == [] (
  SET LZ4_ROOT=%RUN_LOCATION%\lz4
)
if [%MADARA_ROOT%] == [] (
  SET MADARA_ROOT=%RUN_LOCATION%\madara
)
if [%MPC_ROOT%] == [] (
  SET MPC_ROOT=%RUN_LOCATION%\mpc
)
if [%OSC_ROOT%] == [] (
  SET OSC_ROOT=%RUN_LOCATION%\oscpack
)
if [%UNREAL_GAMS_ROOT%] == [] (
  SET UNREAL_GAMS_ROOT=%RUN_LOCATION%\UnrealGAMS
)
if [%VREP_ROOT%] == [] (
  SET VREP_ROOT=%RUN_LOCATION%\vrep
)

echo Environment variables are:
echo   BOOST_ROOT=%BOOST_ROOT%
echo   CAPNP_ROOT=%CAPNP_ROOT%
echo   EIGEN_ROOT=%EIGEN_ROOT%
echo   GAMS_ROOT=%GAMS_ROOT%
echo   LZ4_ROOT=%LZ4_ROOT%
echo   MADARA_ROOT=%MADARA_ROOT%
echo   MPC_ROOT=%MPC_ROOT%
echo   OSC_ROOT=%OSC_ROOT%
echo   UNREAL_GAMS_ROOT=%UNREAL_GAMS_ROOT%
echo   VREP_ROOT=%VREP_ROOT%


IF %PREREQS% EQU 1 (
  echo.
  echo UPDATING CAPNPROTO
  
  IF EXIST %CAPNP_ROOT% (
  
    cd "%CAPNP_ROOT%\c++"
	
    echo git pull
    git pull
    SET CAPNP_REPO_RESULT=%ERRORLEVEL%	
	
  ) ELSE (
  
    echo git clone https://github.com/capnproto/capnproto.git %CAPNP_ROOT%
    git clone https://github.com/capnproto/capnproto.git %CAPNP_ROOT%
    SET CAPNP_REPO_RESULT=%ERRORLEVEL%
    
    SET FORCE_CAPNP=1
  )
  
  IF %FORCE_CAPNP% EQU 1 (
    cd "%CAPNP_ROOT%\c++"
  
    echo cmake -G "Visual Studio 15 2017 Win64"
    cmake -G "Visual Studio 15 2017 Win64"
  
    echo cmake --build . --config Debug
    cmake --build . --config Debug
    SET CAPNP_BUILD_RESULT=%ERRORLEVEL%
  
    echo cmake --build . --config Release
    cmake --build . --config Release
    SET /A CAPNP_BUILD_RESULT+=%ERRORLEVEL%
  )
  
  echo UPDATING MPC
  
  IF NOT EXIST %MPC_ROOT% (
  
    echo git clone --depth 1 https://github.com/DOCGroup/MPC.git %MPC_ROOT%
    git clone --depth 1 https://github.com/DOCGroup/MPC.git %MPC_ROOT%
    SET MPC_REPO_RESULT=%ERRORLEVEL%
	
  ) ELSE (
  
    cd %MPC_ROOT%
    echo git pull
    git pull
    SET MPC_REPO_RESULT=%ERRORLEVEL%
	
  )
  
  echo UPDATING EIGEN
  
  IF NOT EXIST %EIGEN_ROOT% (
  
    echo git clone --single-branch --branch 3.3.4 --depth 1 https://github.com/eigenteam/eigen-git-mirror.git %EIGEN_ROOT%
    git clone --single-branch --branch 3.3.4 --depth 1 https://github.com/eigenteam/eigen-git-mirror.git %EIGEN_ROOT%
    SET EIGEN_REPO_RESULT=%ERRORLEVEL%
	
  ) ELSE (
  
    cd %EIGEN_ROOT%
    echo git pull
    git pull
    SET EIGEN_REPO_RESULT=%ERRORLEVEL%
	
  )
  
  echo UPDATING OSCPACK
  
  IF NOT EXIST %OSC_ROOT% (
  
    echo git clone https://github.com/jredmondson/oscpack %OSC_ROOT%
    git clone https://github.com/jredmondson/oscpack %OSC_ROOT%
    SET OSC_REPO_RESULT=%ERRORLEVEL%
	
    SET FORCE_OSC=1
  ) ELSE (
  
    cd %OSC_ROOT%
    echo git pull
    git pull
    SET OSC_REPO_RESULT=%ERRORLEVEL%
	
  )
  
  IF %FORCE_OSC% EQU 1 (
  
    cd "%OSC_ROOT%"
  
    echo cmake -G "Visual Studio 15 2017 Win64"
    cmake -G "Visual Studio 15 2017 Win64"
  
    echo cmake --build . --config Debug
    cmake --build . --config Debug
    SET OSC_BUILD_RESULT=%ERRORLEVEL%
  
    echo cmake --build . --config Release
    cmake --build . --config Release
    SET /A OSC_BUILD_RESULT+=%ERRORLEVEL%
  )
  
  echo UPDATING BOOST
  
  IF NOT EXIST %BOOST_ROOT% (
  
    echo git clone --recursive https://github.com/boostorg/boost.git %BOOST_ROOT%
    git clone --recursive https://github.com/boostorg/boost.git %BOOST_ROOT%
	  SET BOOST_REPO_RESULT=%ERRORLEVEL%
    
    SET FORCE_BOOST=1
	
  ) ELSE (
  
    cd %BOOST_ROOT%
    echo git pull
    git pull
	  SET BOOST_REPO_RESULT=%ERRORLEVEL%
  )
  
  IF %FORCE_BOOST% EQU 1 (
    echo Updating Boost submodules
    echo git submodule update --init --recursive
    git submodule update --init --recursive

    echo cd %BOOST_ROOT%
    cd %BOOST_ROOT%
    echo .\bootstrap.bat --with-library=system,python,filesystem
    .\bootstrap.bat --with-library=system,python,filesystem
    echo cd %BOOST_ROOT%
    cd %BOOST_ROOT%
    echo b2.exe --with-system --with-python --with-filesystem
    .\b2.exe --with-system --with-python --with-filesystem
    SET BOOST_BUILD_RESULT=%ERRORLEVEL%
  )
  
  echo   BOOST_REPO_RESULT=%BOOST_REPO_RESULT%
  echo   BOOST_BUILD_RESULT=%BOOST_BUILD_RESULT%
  echo   CAPNP_REPO_RESULT=%CAPNP_REPO_RESULT%
  echo   CAPNP_BUILD_RESULT=%CAPNP_BUILD_RESULT%
  echo   EIGEN_REPO_RESULT=%EIGEN_REPO_RESULT%
  echo   OSC_REPO_RESULT=%OSC_REPO_RESULT%
  echo   OSC_BUILD_RESULT=%OSC_BUILD_RESULT%
  echo   MPC_REPO_RESULT=%MPC_REPO_RESULT%

)

cd %RUN_LOCATION%

IF %MADARA% EQU 1 (
  echo.
  echo Generating MADARA project with docs=%DOCS%, java=%JAVA%, tests=%TESTS% and tutorials=%TUTORIALS%
  cd "%MADARA_ROOT%"
  "%ACE_ROOT%\bin\mwc.pl" -type %vs_version% -features nothreadlocal=1,tests=%TESTS%,tutorials=%TUTORIALS%,java=%JAVA%,docs=%DOCS% MADARA.mwc
  echo Building MADARA library for Debug target with tests=%TESTS%
  msbuild "%MADARA_ROOT%\MADARA.sln" /maxcpucount /t:Rebuild /clp:NoSummary;NoItemAndPropertyList;ErrorsOnly /verbosity:quiet /nologo /p:Configuration=Debug;Platform=X64 /target:Madara
  echo Building MADARA for Release target with tests=%TESTS%
  msbuild "%MADARA_ROOT%\MADARA.sln" /maxcpucount /t:Rebuild /clp:NoSummary;NoItemAndPropertyList;ErrorsOnly /verbosity:quiet /nologo /p:Configuration=Release;Platform=X64
)

IF %GAMS% EQU 1 (
  echo.
  echo Generating GAMS project with docs=%DOCS%, java=%JAVA%, tests=%TESTS% and vrep=%VREP%
  cd "%GAMS_ROOT%"
  "%ACE_ROOT%\bin\mwc.pl" -type %vs_version% -features nothreadlocal=1,docs=%DOCS%,vrep=%VREP%,tests=%TESTS%,java=%JAVA% gams.mwc
  echo Building GAMS library for Debug target with tests=%TESTS% and vrep=%VREP%
  msbuild "gams.sln" /maxcpucount /t:Rebuild /clp:NoSummary;NoItemAndPropertyList;ErrorsOnly /verbosity:quiet /nologo /p:Configuration=Debug;Platform=X64 /target:gams
  echo Building GAMS for Release target with tests=%TESTS% and vrep=%VREP%
  msbuild "gams.sln" /maxcpucount /t:Rebuild /clp:NoSummary;NoItemAndPropertyList;ErrorsOnly /verbosity:quiet /nologo /p:Configuration=Release;Platform=X64
)

if %VREP_CONFIG% EQU 1 (
  echo.
  echo Configuring 20 ports in VREP
  %GAMS_ROOT%\scripts\simulation\remoteApiConnectionsGen.pl 19905 20
)

:END_OF_SCRIPT

IF NOT EXIST "%HOMEDRIVE%%HOMEPATH%"\.gams (
  mkdir "%HOMEDRIVE%%HOMEPATH%"\.gams
)

echo BOOST_ROOT=%BOOST_ROOT% > "%HOMEDRIVE%%HOMEPATH%"\.gams\env.bat
echo CAPNP_ROOT=%CAPNP_ROOT% >> "%HOMEDRIVE%%HOMEPATH%"\.gams\env.bat
echo EIGEN_ROOT=%EIGEN_ROOT% >> "%HOMEDRIVE%%HOMEPATH%"\.gams\env.bat
echo GAMS_ROOT=%GAMS_ROOT% >> "%HOMEDRIVE%%HOMEPATH%"\.gams\env.bat
echo LZ4_ROOT=%LZ4_ROOT% >> "%HOMEDRIVE%%HOMEPATH%"\.gams\env.bat
echo MADARA_ROOT=%MADARA_ROOT% >> "%HOMEDRIVE%%HOMEPATH%"\.gams\env.bat
echo MPC_ROOT=%MPC_ROOT% >> "%HOMEDRIVE%%HOMEPATH%"\.gams\env.bat
echo OSC_ROOT=%OSC_ROOT% >> "%HOMEDRIVE%%HOMEPATH%"\.gams\env.bat
echo UNREAL_GAMS_ROOT=%UNREAL_GAMS_ROOT% >> "%HOMEDRIVE%%HOMEPATH%"\.gams\env.bat
echo VREP_ROOT=%VREP_ROOT% >> "%HOMEDRIVE%%HOMEPATH%"\.gams\env.bat
echo PATH=%%PATH%%:%%MPC_ROOT%%:%%CAPNP_ROOT%%\c++\src\capnp\Release:%%MADARA_ROOT%%\lib:%%MADARA_ROOT%%\bin:%%GAMS_ROOT%%\lib >> "%HOMEDRIVE%%HOMEPATH%"\.gams\env.bat

echo
echo Environment saved to "%HOMEDRIVE%%HOMEPATH%"\.gams\env.bat
echo To compile MADARA/GAMS programs, make sure your environment is updated




endlocal

cd %RUN_LOCATION%

@echo on


