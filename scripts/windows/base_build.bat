@echo off

set RUN_LOCATION=%CD%

setlocal enableDelayedExpansion

SET vrep=0
SET vrep_config=0
SET java=0
SET ace=0
SET gams=0
SET madara=0
SET tests=0
SET tutorials=0
SET docs=0

FOR %%x in (%*) do (
   
   IF "%%x" == "ace" (
     echo Build will enable ACE
     SET ace=1
   ) ELSE IF "%%x" == "docs" (
     echo Build will enable doxygen documentation
     SET docs=1
   ) ELSE IF "%%x" == "gams" (
     echo Build will enable GAMS
     SET gams=1
   ) ELSE IF "%%x" == "java" (
     echo Build will enable Java support
     SET java=1
   ) ELSE IF "%%x" == "madara" (
     echo Build will enable MADARA
     SET madara=1
   ) ELSE IF "%%x" == "tests" (
     echo Build will enable tests
     SET tests=1
   ) ELSE IF "%%x" == "tutorials" (
     echo Build will enable tutorials
     SET tutorials=1
   ) ELSE IF "%%x" == "vrep" (
     echo Build will enable vrep
     SET vrep=1
   ) ELSE IF "%%x" == "vrep_config" (
     echo Build will configure vrep
     SET vrep_config=1
   ) ELSE IF "%%x" == "vrep-config" (
     echo Build will configure vrep
     SET vrep_config=1
   ) ELSE (
     echo ERROR: Bad argument "%%x"
     echo   Appropriate arguments are any combination of 
     echo     ace         Build ACE
     echo     docs        Enable doxygen documentation generation
     echo     gams        Build GAMS
     echo     java        Enable Java support
     echo     madara      Build MADARA
     echo     tests       Build tests
     echo     tutorials   Build tutorials
     echo     vrep        Enable VREP support
     echo     vrep-config Configure VREP installation for 20 agents

     GOTO  END_OF_SCRIPT
   )
)

echo Building options are:
echo   ace=%ace%
echo   docs=%docs%
echo   gams=%gams%
echo   madara=%madara%
echo   tests=%tests%
echo   tutorials=%tutorials%
echo   vrep=%vrep%

IF %ace% EQU 1 (
  echo.
  echo Generating ACE project
  cd "%ACE_ROOT%\ace"
  echo #include "ace/config-win32.h" > config.h
  "%ACE_ROOT%\bin\mwc.pl" -type vc12 ace.mwc
  echo Building ACE library for Debug target
  msbuild "ace.sln" /maxcpucount /t:Rebuild /clp:NoSummary;NoItemAndPropertyList;ErrorsOnly /verbosity:quiet /nologo /p:Configuration=Debug;Platform=X64 /target:ACE
  echo Building ACE for Release target
  msbuild "ace.sln" /maxcpucount /t:Rebuild /clp:NoSummary;NoItemAndPropertyList;ErrorsOnly /verbosity:quiet /nologo /p:Configuration=Release;Platform=X64 /target:ACE
)
IF %madara% EQU 1 (
  echo.
  echo Generating MADARA project with docs=%docs%, java=%java%, tests=%tests% and tutorials=%tutorials%
  cd "%MADARA_ROOT%"
  "%ACE_ROOT%\bin\mwc.pl" -type vc12 -features tests=%tests%,tutorials=%tutorials%,java=%java%,docs=%docs% MADARA.mwc
  echo Building MADARA library for Debug target with tests=%tests%
  msbuild "%MADARA_ROOT%\MADARA.sln" /maxcpucount /t:Rebuild /clp:NoSummary;NoItemAndPropertyList;ErrorsOnly /verbosity:quiet /nologo /p:Configuration=Debug;Platform=X64 /target:Madara
  echo Building MADARA for Release target with tests=%tests%
  msbuild "%MADARA_ROOT%\MADARA.sln" /maxcpucount /t:Rebuild /clp:NoSummary;NoItemAndPropertyList;ErrorsOnly /verbosity:quiet /nologo /p:Configuration=Release;Platform=X64
)

IF %gams% EQU 1 (
  echo.
  echo Generating GAMS project with docs=%docs%, java=%java%, tests=%tests% and vrep=%vrep%
  cd "%GAMS_ROOT%"
  "%ACE_ROOT%\bin\mwc.pl" -type vc12 -features docs=%docs%,vrep=%vrep%,tests=%tests%,java=%java% gams.mwc
  echo Building GAMS library for Debug target with tests=%tests% and vrep=%vrep%
  msbuild "gams.sln" /maxcpucount /t:Rebuild /clp:NoSummary;NoItemAndPropertyList;ErrorsOnly /verbosity:quiet /nologo /p:Configuration=Debug;Platform=X64 /target:gams
  echo Building GAMS for Release target with tests=%tests% and vrep=%vrep%
  msbuild "gams.sln" /maxcpucount /t:Rebuild /clp:NoSummary;NoItemAndPropertyList;ErrorsOnly /verbosity:quiet /nologo /p:Configuration=Release;Platform=X64
)

if %vrep_config% EQU 1 (
  echo.
  echo Configuring 20 ports in VREP
  %GAMS_ROOT%\scripts\simulation\remoteApiConnectionsGen.pl 19905 20
)

:END_OF_SCRIPT

endlocal

cd %RUN_LOCATION%

@echo on


