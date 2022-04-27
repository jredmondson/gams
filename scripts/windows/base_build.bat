@echo off

set RUN_LOCATION=%CD%

setlocal enableDelayedExpansion

SET VREP=0
SET VREP_CONFIG=0
SET JAVA=0
SET ACE=0
SET CAPNP=0
SET CLEAN=1
SET CUDA=0
SET EIGEN=0
SET GAMS=0
SET MADARA=0
SET OSC=0
SET TESTS=0
SET TUTORIALS=0
SET OPENCV=0
SET DOCS=0
::SET VS_VERSION="vs2017"
::SET CMAKE_GEN=Visual Studio 15 2017
SET VS_VERSION="vs2022"
SET CMAKE_GEN=Visual Studio 17 2022
SET PREREQS=0
SET SETENV=0
SET SETPATH=0
SET BOOST_STATIC_LIB_PREFIX=lib
SET BOOST_TOOLSET=vc142
SET BOOST_VERSION=1_73
SET BOOST_ARCH=x64

SET CAPNP_REPO_RESULT=0
SET CAPNP_BUILD_RESULT=0

SET FORCE_BOOST=0
SET FORCE_CAPNP=0
SET FORCE_OPENCV=0
SET FORCE_OSC=0

ECHO Parsing arguments

FOR %%x in (%*) do (

    IF "%%x" == "capnp" (
     SET CAPNP=1
   ) ELSE IF "%%x" == "cuda" (
     echo When appropriate, build CUDA GPU-enhancement support
     SET CUDA=1
   ) ELSE IF "%%x" == "docs" (
     echo Build will enable doxygen documentation
     SET DOCS=1
   ) ELSE IF "%%x" == "eigen" (
     echo Download the latest Eigen header library
     SET EIGEN=1
   ) ELSE IF "%%x" == "forceboost" (
     echo Prereqs will force Boost install
     SET FORCE_BOOST=1
     SET PREREQS=1
   ) ELSE IF "%%x" == "forcecapnp" (
     echo Prereqs will force capnp build
     SET FORCE_CAPNP=1
     SET PREREQS=1
   ) ELSE IF "%%x" == "forceopencv" (
     echo Prereqs will force opencv build
     SET FORCE_OPENCV=1
     SET OPENCV=1
   ) ELSE IF "%%x" == "forceosc" (
     echo Prereqs will force OSC build
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
   ) ELSE IF "%%x" == "opencv" (
     echo Build will include opencv
     SET OPENCV=1
   ) ELSE IF "%%x" == "osc" (
     echo Build will include open stage control
     SET OSC=1
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
     echo Build will use Visual Studio 2017
     SET VS_VERSION=vs2017
   ) ELSE IF "%%x" == "vs2019" (
     echo Build will use Visual Studio 2019
     SET VS_VERSION=vs2019
   ) ELSE IF "%%x" == "vs2022" (
     echo Build will use Visual Studio 2022
     SET VS_VERSION=vs2022
   ) ELSE IF "%%x" == "setenv" (
     echo Build will set environment variables for you
     SET SETENV=1
   ) ELSE IF "%%x" == "setpath" (
     echo Build will set path environment variables for you.
	 echo Warning: this is dangerous. A copy will be saved of
	 echo your current PATH in %HOMEDRIVE%%HOMEPATH%\.gams\oldpath.bat
     SET SETENV=1
     SET SETPATH=1
   ) ELSE (
     echo ERROR: Bad argument "%%x"
     echo   Appropriate arguments are any combination of 
     echo     capnp       Enable capnp custom type support
     echo     docs        Enable doxygen documentation generation
     echo     eigen       Download latest Eigen library
     echo     forceboost  Force boost installation
     echo     forcecapnp  Force capnp installation
     echo     forceosc    Force open stage control installation
     echo     gams        Build GAMS
     echo     java        Enable Java support
     echo     madara      Build MADARA
     echo     opencv      Build opencv and opencv_contrib
     echo     osc         Build osc
     echo     prereqs     Build prereqs
     echo     setenv      set environment variables
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
echo   cuda=%CUDA%
echo   docs=%DOCS%
echo   eigen=%EIGEN%
echo   gams=%GAMS%
echo   madara=%MADARA%
echo   opencv=%OPENCV%
echo   OSC=%OSC%
echo   prereqs=%PREREQS%
echo   setenv=%SETENV%
echo   setpath=%SETPATH%
echo   tests=%TESTS%
echo   tutorials=%TUTORIALS%
echo   vrep=%VREP%

:: Create environment variables if they don't exist
if ["%BOOST_ROOT%"] == [""] (
  SET BOOST_ROOT=%RUN_LOCATION%\boost
)

if ["%BOOST_STATIC_LIB_PREFIX%"] == [""] (
  SET BOOST_STATIC_LIB_PREFIX=lib
)

if ["%BOOST_TOOLSET%"] == [""] (
  SET BOOST_TOOLSET=vc142
)

if ["%BOOST_VERSION%"] == [""] (
  SET BOOST_VERSION=1_73
)

if ["%BOOST_ARCH%"] == [""] (
  SET BOOST_ARCH=x64
)

if ["%CAPNP_ROOT%"] == [""] (
  SET CAPNP_ROOT=%RUN_LOCATION%\capnproto
)
if ["%EIGEN_ROOT%"] == [""] (
  SET EIGEN_ROOT=%RUN_LOCATION%\eigen
)
if ["%GAMS_ROOT%"] == [""] (
  SET GAMS_ROOT=%RUN_LOCATION%\gams
)
if ["%LZ4_ROOT%"] == [""] (
  SET LZ4_ROOT=%RUN_LOCATION%\lz4
)
if ["%MADARA_ROOT%"] == [""] (
  SET MADARA_ROOT=%RUN_LOCATION%\madara
)
if ["%MPC_ROOT%"] == [""] (
  SET MPC_ROOT=%RUN_LOCATION%\mpc
)
if ["%OPENCV_ROOT%"] == [""] (
  SET OPENCV_ROOT=%RUN_LOCATION%\opencv
)
if ["%OPENCV_CONTRIB_ROOT%"] == [""] (
  SET OPENCV_CONTRIB_ROOT=%RUN_LOCATION%\opencv_contrib
)
if ["%OSC_ROOT%"] == [""] (
  SET OSC_ROOT=%RUN_LOCATION%\oscpack
)
if ["%UNREAL_GAMS_ROOT%"] == [""] (
  SET UNREAL_GAMS_ROOT=%RUN_LOCATION%\UnrealGAMS
)
if ["%VREP_ROOT%"] == [""] (
  SET VREP_ROOT=%RUN_LOCATION%\vrep
)

if ["%VS_VERSION%"] == ["vs2017"] (
  SET CMAKE_GEN=Visual Studio 15 2017
  SET BOOST_TOOLSET=vc142
  echo VS2017 detected. CMAKE_GEN=%CMAKE_GEN%. BOOST_TOOLSET=%BOOST_TOOLSET%
)

if ["%VS_VERSION%"] == ["vs2019"] (
  SET CMAKE_GEN=Visual Studio 16 2019
  SET BOOST_TOOLSET=vc142
  echo VS2019 detected. CMAKE_GEN=%CMAKE_GEN%. BOOST_TOOLSET=%BOOST_TOOLSET%
)


echo Environment variables are:
echo   BOOST_ROOT=%BOOST_ROOT%
echo   CAPNP_ROOT=%CAPNP_ROOT%
echo   EIGEN_ROOT=%EIGEN_ROOT%
echo   GAMS_ROOT=%GAMS_ROOT%
echo   LZ4_ROOT=%LZ4_ROOT%
echo   MADARA_ROOT=%MADARA_ROOT%
echo   OPENCV_ROOT=%OPENCV_ROOT%
echo   OSC_ROOT=%OSC_ROOT%
echo   UNREAL_GAMS_ROOT=%UNREAL_GAMS_ROOT%
echo   VREP_ROOT=%VREP_ROOT%
echo   CMAKE_GEN=%CMAKE_GEN%
echo   BOOST_TOOLSET=%BOOST_TOOLSET%

IF %PREREQS% EQU 1 (

  SET FORCE_BOOST=1
  
  IF %GAMS% EQU 1 (
    echo ENABLING GAMS PREREQS
    SET MADARA=1
    SET EIGEN=1
    SET OSC=1
    SET FORCE_OSC=1
  )

  IF %EIGEN% EQU 1 (
    echo UPDATING EIGEN
    
    IF NOT EXIST %EIGEN_ROOT% (
    
      echo EIGEN does not exist. Cloning from github.
      echo git clone --single-branch --branch 3.3.4 --depth 1 https://github.com/eigenteam/eigen-git-mirror.git %EIGEN_ROOT%
      git clone --single-branch --branch 3.3.4 --depth 1 https://github.com/eigenteam/eigen-git-mirror.git %EIGEN_ROOT%
      SET EIGEN_REPO_RESULT=%ERRORLEVEL%
    
    ) ELSE (
    
      echo EIGEN exists. Pulling latest version
      cd %EIGEN_ROOT%
      echo git pull
      git pull
      SET EIGEN_REPO_RESULT=%ERRORLEVEL%
    
    )
  )
  
  IF %OSC% EQU 1 (
    IF NOT EXIST %OSC_ROOT% (
    
      echo OSC does not exist. Cloning from github.
      echo git clone https://github.com/jredmondson/oscpack %OSC_ROOT%
      git clone https://github.com/jredmondson/oscpack %OSC_ROOT%
      SET OSC_REPO_RESULT=%ERRORLEVEL%
    
      SET FORCE_OSC=1
    ) ELSE (
    
      echo OSC exists. Pulling latest version
      cd %OSC_ROOT%
      echo git pull
      git pull
      SET OSC_REPO_RESULT=%ERRORLEVEL%
    
    )
    
    IF %FORCE_OSC% EQU 1 (
    
      cd "%OSC_ROOT%"
    
      echo cmake -G "%CMAKE_GEN%" -A "x64"  -DCMAKE_INSTALL_PREFIX="%OSC_ROOT%\install"
      cmake -G "%CMAKE_GEN%" -A "x64" -DCMAKE_INSTALL_PREFIX="%OSC_ROOT%\install"
    
      echo cmake --build . --config Debug
      cmake --build . --config debug
      cmake --build . --target install --config debug
      SET OSC_BUILD_RESULT=%ERRORLEVEL%
    
      echo cmake --build . --config Release
      cmake --build . --config release
      cmake --build . --target install --config release
      SET /A OSC_BUILD_RESULT+=%ERRORLEVEL%
    )
  )
  
  echo UPDATING BOOST
  
  IF NOT EXIST %BOOST_ROOT% (
  
    echo BOOST does not exist. Cloning from github.
    echo git clone --recursive https://github.com/boostorg/boost.git %BOOST_ROOT%
    git clone --recursive https://github.com/boostorg/boost.git %BOOST_ROOT%
    SET BOOST_REPO_RESULT=%ERRORLEVEL%
    
  ) ELSE (
  
    echo BOOST exists. Pulling latest version
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
    echo .\bootstrap.bat --with-library=system,python
    .\bootstrap.bat --with-library=system,python
    echo cd %BOOST_ROOT%
    cd %BOOST_ROOT%
    echo b2.exe --with-system --with-python
    .\b2.exe --with-system --with-python
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

IF %CAPNP% EQU 1 (
  echo.
  echo UPDATING CAPNPROTO
  
  SET FORCE_CAPNP=1
    
  IF EXIST %CAPNP_ROOT% (
  
    cd "%CAPNP_ROOT%\c++"
  
    echo CAPNP exists. Pulling latest version
    echo git pull
    git pull
    SET CAPNP_REPO_RESULT=%ERRORLEVEL%	
  
  ) ELSE (
  
    echo CAPNP does not exist. Cloning from github.
    echo git clone https://github.com/capnproto/capnproto.git %CAPNP_ROOT%
    git clone https://github.com/capnproto/capnproto.git %CAPNP_ROOT%
    SET CAPNP_REPO_RESULT=%ERRORLEVEL%
    
  )

  IF %FORCE_CAPNP% EQU 1 (
    cd "%CAPNP_ROOT%\c++"
  
    echo if cmake fails, make sure you have a development branch version
  echo from https://cmake.org/files/dev/
  
    echo cmake -G "%CMAKE_GEN%" -A "x64"
    cmake -G "%CMAKE_GEN%" -A "x64"
  
    echo cmake --build . --config Debug
    cmake --build . --config Debug
    SET CAPNP_BUILD_RESULT=%ERRORLEVEL%
  
    echo cmake --build . --config Release
    cmake --build . --config Release
    SET /A CAPNP_BUILD_RESULT+=%ERRORLEVEL%
  )
)
  

IF %OPENCV% EQU 1 (
  echo UPDATING OPENCV
  
	SET FORCE_OPENCV=1
	
  IF NOT EXIST "%OPENCV_ROOT%" (
  
    echo OPENCV does not exist. Cloning from github.
    echo git clone https://github.com/opencv/opencv.git %OPENCV_ROOT%
    git clone https://github.com/opencv/opencv.git %OPENCV_ROOT%
    SET OPENCV_REPO_RESULT=%ERRORLEVEL%
	
  ) ELSE (
  
    echo OPENCV exists. Pulling latest version
    cd %OPENCV_ROOT%
    echo git pull
    git pull
    SET OPENCV_REPO_RESULT=%ERRORLEVEL%
	
  )
  
  IF NOT EXIST "%OPENCV_CONTRIB_ROOT%" (
  
    echo OPENCV does not exist. Cloning from github.
    echo git clone https://github.com/opencv/opencv_contrib %OPENCV_CONTRIB_ROOT%
    git clone https://github.com/opencv/opencv_contrib %OPENCV_CONTRIB_ROOT%
    SET OPENCV_CONTRIB_REPO_RESULT=%ERRORLEVEL%
	
  ) ELSE (
  
    echo OPENCV exists. Pulling latest version
    cd %OPENCV_CONTRIB_ROOT%
    echo git pull
    git pull
    SET OPENCV_CONTRIB_REPO_RESULT=%ERRORLEVEL%
	
  )
  
  IF %FORCE_OPENCV% EQU 1 (
    echo BUILDING OPENCV
    IF NOT EXIST "%OPENCV_ROOT%\build" (
      echo .. ensuring build and install dirs exist in OPENCV_ROOT
      mkdir "%OPENCV_ROOT%\build"
      mkdir "%OPENCV_ROOT%\install"
    )
  
    cd "%OPENCV_ROOT%\build"
  
    IF %CUDA% EQU 0 (
      echo ... disabling CUDA
      SET CMAKE_OPTIONS="-DBUILD_PERF_TESTS:BOOL=OFF -DBUILD_TESTS:BOOL=OFF -DBUILD_DOCS:BOOL=OFF  -DWITH_CUDA:BOOL=OFF -DBUILD_EXAMPLES:BOOL=OFF -DINSTALL_CREATE_DISTRIB=ON"
    ) ELSE (
      echo ... enabling CUDA
      SET CMAKE_OPTIONS="-DBUILD_PERF_TESTS:BOOL=OFF -DBUILD_TESTS:BOOL=OFF -DBUILD_DOCS:BOOL=OFF  -DWITH_CUDA:BOOL=ON -DBUILD_EXAMPLES:BOOL=OFF -DINSTALL_CREATE_DISTRIB=ON"
    )

    echo ... generating makefiles for %CMAKE_GEN%
    echo cmake -G "%CMAKE_GEN%" -A "x64" %CMAKE_OPTIONS% -DOPENCV_EXTRA_MODULES_PATH="%OPENCV_CONTRIB_ROOT%/modules" -DCMAKE_INSTALL_PREFIX="%OPENCV_ROOT%\install" "%OPENCV_ROOT%"
    cmake -G "%CMAKE_GEN%" -A "x64" %CMAKE_OPTIONS% -DOPENCV_EXTRA_MODULES_PATH="%OPENCV_CONTRIB_ROOT%/modules" -DCMAKE_INSTALL_PREFIX="%OPENCV_ROOT%\install" "%OPENCV_ROOT%"

    echo ... build debug libs with %CMAKE_GEN%
    cmake --build .  --config debug
	
    echo ... build release libs with %CMAKE_GEN%
    cmake --build .  --config release
    cmake --build .  --target install --config release
    cmake --build .  --target install --config debug
  )
)

cd %RUN_LOCATION%

echo Check for installing MADARA (MADARA=%MADARA%)

IF %MADARA% EQU 1 (
  
  IF NOT EXIST "%MADARA_ROOT%" (
  
    echo MADARA does not exist. Cloning from github.
    echo git clone https://github.com/jredmondson/madara "%MADARA_ROOT%"
    git clone https://github.com/jredmondson/madara "%MADARA_ROOT%"
	SET MADARA_REPO_RESULT=%ERRORLEVEL%
	cd "%MADARA_ROOT%"
	git checkout master
	
  ) ELSE (
  
    echo MADARA exists. Pulling latest version
    cd "%MADARA_ROOT%"
	git checkout master
    echo git pull
    git pull
	SET MADARA_REPO_RESULT=%ERRORLEVEL%
  )
  
  echo Generating MADARA project with docs=%DOCS%, java=%JAVA%, tests=%TESTS% and tutorials=%TUTORIALS%
  cd "%MADARA_ROOT%"
  
  mkdir build
  mkdir install
  
  cd build
  
  cmake -G "%CMAKE_GEN%" -A "x64" -Dmadara_TESTS=%TESTS% -DCMAKE_INSTALL_PREFIX=..\install ..
  echo ... build debug libs with %CMAKE_GEN%
  cmake --build .  --config debug
	
  echo ... build release libs with %CMAKE_GEN%
  cmake --build .  --config release
  echo ... installing to %MADARA_ROOT%\install
  cmake --build .  --target install --config release
  cmake --build .  --target install --config debug
  
)

echo Check for installing GAMS

cd %RUN_LOCATION%

IF %GAMS% EQU 1 (

  IF NOT EXIST %GAMS_ROOT% (
  
    echo GAMS does not exist. Cloning from github.
    echo git clone https://github.com/jredmondson/gams %GAMS_ROOT%
    git clone https://github.com/jredmondson/gams %GAMS_ROOT%
	SET GAMS_REPO_RESULT=%ERRORLEVEL%
	cd %GAMS_ROOT%
	git checkout master
	
  ) ELSE (
    echo GAMS exists. Pulling latest version
    cd %GAMS_ROOT%
	git checkout master
    echo git pull
    git pull
	SET GAMS_REPO_RESULT=%ERRORLEVEL%
  )
  
  echo.
  echo Generating GAMS project with docs=%DOCS%, java=%JAVA%, tests=%TESTS%
  
  mkdir build
  mkdir install
  
  cd build
  cmake -G "%CMAKE_GEN%" -A "x64" -DCMAKE_INSTALL_PREFIX="..\install" -Dgams_TESTS=%TESTS% -DCMAKE_PREFIX_PATH=%MADARA_ROOT%\install ..
  echo ... build debug libs with %CMAKE_GEN%
  cmake --build .  --config debug
	
  echo ... build release libs with %CMAKE_GEN%
  cmake --build .  --config release
  echo ... installing to %GAMS_ROOT%\install
  cmake --build .  --target install --config release
  cmake --build .  --target install --config debug
)

echo Check for configuring VREP

if %VREP_CONFIG% EQU 1 (
  echo.
  echo Configuring 20 ports in VREP
  %GAMS_ROOT%\scripts\simulation\remoteApiConnectionsGen.pl 19905 20
)

echo Check for existing gams paths

IF NOT EXIST "%HOMEDRIVE%%HOMEPATH%"\.gams (
  echo Creating .gams directory in %HOMEDRIVE%%HOMEPATH%
  mkdir "%HOMEDRIVE%%HOMEPATH%"\.gams
  
)

IF %SETPATH% EQU 1 (
  IF EXIST "%HOMEDRIVE%%HOMEPATH%\.gams\oldpath.bat" (
    echo Appending path information to %HOMEDRIVE%%HOMEPATH%\.gams\oldpath.bat
    echo SET PATH="%PATH%" >> "%HOMEDRIVE%%HOMEPATH%\.gams\oldpath.bat"
  ) ELSE (
    echo Creating path information in %HOMEDRIVE%%HOMEPATH%\.gams\oldpath.bat
    echo SET PATH="%PATH%" >> "%HOMEDRIVE%%HOMEPATH%\.gams\oldpath.bat"
  )
)
  
echo Writing environment variables for build to %HOMEDRIVE%%HOMEPATH%\.gams\env.bat
echo SET BOOST_ROOT=%BOOST_ROOT% > "%HOMEDRIVE%%HOMEPATH%\.gams\env.bat"
echo SET CAPNP_ROOT=%CAPNP_ROOT% >> "%HOMEDRIVE%%HOMEPATH%\.gams\env.bat"
echo SET EIGEN_ROOT=%EIGEN_ROOT% >> "%HOMEDRIVE%%HOMEPATH%\.gams\env.bat"
echo SET GAMS_ROOT=%GAMS_ROOT% >> "%HOMEDRIVE%%HOMEPATH%\.gams\env.bat"
echo SET LZ4_ROOT=%LZ4_ROOT% >> "%HOMEDRIVE%%HOMEPATH%\.gams\env.bat"
echo SET MADARA_ROOT=%MADARA_ROOT% >> "%HOMEDRIVE%%HOMEPATH%\.gams\env.bat"
echo SET MPC_ROOT=%MPC_ROOT% >> "%HOMEDRIVE%%HOMEPATH%\.gams\env.bat"
echo SET OSC_ROOT=%OSC_ROOT% >> "%HOMEDRIVE%%HOMEPATH%\.gams\env.bat"
echo SET OPENCV_ROOT=%OPENCV_ROOT% >> "%HOMEDRIVE%%HOMEPATH%\.gams\env.bat"
echo SET UNREAL_GAMS_ROOT=%UNREAL_GAMS_ROOT% >> "%HOMEDRIVE%%HOMEPATH%\.gams\env.bat"
echo SET VREP_ROOT=%VREP_ROOT% >> "%HOMEDRIVE%%HOMEPATH%\.gams\env.bat"
echo SET PATH=%%PATH%%;%%MPC_ROOT%%;%%CAPNP_ROOT%%\c++\src\capnp\Release;%%MADARA_ROOT%%\lib;%%MADARA_ROOT%%\bin;%%GAMS_ROOT%%\lib >> "%HOMEDRIVE%%HOMEPATH%\.gams\env.bat"

echo Checking if variables should be set to outside environment


IF %SETENV% EQU 1 (

  echo Setting local environment variables in your terminal
  SET BOOST_ROOT=%BOOST_ROOT%
  SET CAPNP_ROOT=%CAPNP_ROOT%
  SET EIGEN_ROOT=%EIGEN_ROOT%
  SET GAMS_ROOT=%GAMS_ROOT%
  SET LZ4_ROOT=%LZ4_ROOT%
  SET MADARA_ROOT=%MADARA_ROOT%
  SET MPC_ROOT=%MPC_ROOT%
  SET OPENCV_ROOT=%OPENCV_ROOT%
  SET OPENCV_CONTRIB_ROOT=%OPENCV_CONTRIB_ROOT%
  SET OSC_ROOT=%OSC_ROOT%
  SET UNREAL_GAMS_ROOT=%UNREAL_GAMS_ROOT%
  SET VREP_ROOT=%VREP_ROOT%
  
  echo Setting environment variables in your permanent environment
  SETX BOOST_ROOT %BOOST_ROOT%
  SETX BOOST_TOOLSET %BOOST_TOOLSET%
  SETX BOOST_VERSION %BOOST_VERSION%
  SETX BOOST_STATIC_LIB_PREFIX %BOOST_STATIC_LIB_PREFIX%
  SETX CAPNP_ROOT %CAPNP_ROOT%
  SETX EIGEN_ROOT %EIGEN_ROOT%
  SETX GAMS_ROOT %GAMS_ROOT%
  SETX LZ4_ROOT %LZ4_ROOT%
  SETX MADARA_ROOT %MADARA_ROOT%
  SETX MPC_ROOT %MPC_ROOT%
  SETX OPENCV_ROOT %OPENCV_ROOT%
  SETX OPENCV_CONTRIB_ROOT %OPENCV_CONTRIB_ROOT%
  SETX OSC_ROOT %OSC_ROOT%
  SETX UNREAL_GAMS_ROOT %UNREAL_GAMS_ROOT%
  SETX VREP_ROOT %VREP_ROOT%
  
  IF NOT "%PATHED_ROOT%" == "" (
  
    SETX PATHED_ROOT %PATHED_ROOT%
    
    echo Setting PATH. Please allow Admin access.
    
    %GAMS_ROOT%\scripts\windows\admin.bat %GAMS_ROOT%\scripts\windows\set_path.bat
    
  ) ELSE (
  
    echo All variables but %%PATH%% are set permanently. To update PATH,
    echo go to Start->Edit Environment Variables and set your PATH to include
    echo the following
    echo
    echo   %%MADARA_ROOT%%\install\lib
    echo   %%MADARA_ROOT%%\install\bin
    echo   %%GAMS_ROOT%%\install\lib
    echo   %%GAMS_ROOT%%\install\bin
	echo
  )
  
  echo Environment variables have been set. Feel free to close this terminal.
  
) ELSE (
  echo Not permanently setting environment
)

endlocal

:END_OF_SCRIPT

echo
echo Environment saved to "%HOMEDRIVE%%HOMEPATH%\.gams\env.bat"
echo To compile MADARA/GAMS programs, make sure your environment is updated




cd %RUN_LOCATION%

@echo on


