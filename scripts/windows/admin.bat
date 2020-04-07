@echo off

(if '%1'=='' SET $Help$=Yes)&(if '%1'=='?' SET $Help$=Yes)&(if '%1'=='/?' SET $Help$=Yes)&(if /I '%1'=='/HELP' SET $Help$=Yes)&(if /I '%1'=='-HELP' SET $Help$=Yes)&(if /I '%1'=='/INFO' SET $Help$=Yes)

if '%$Help$%'=='Yes' if exist admin.bat  (SET $Help_BAT$=admin.bat) else (FOR /F %%I IN ("admin.bat") DO (SET $Help_BAT$=%%~$PATH:I))

if '%$Help$%'=='Yes' (SET $Help$=&cls&MORE /C /E +85 "%$Help_BAT$%"&SET $Help_BAT$=&pause&goto:eof)

 

::  A D M I N I S T R A T O R   - Automatically get admin rights for script batch. Paste this on top:    net session >nul 2>nul&if errorlevel 1  admin "%~0" %*

::                                Also keep Batch directory localisation and then set variable:   PATH_BAT

::                                if earlier variable "ShowAdminInfo" is empty (not defined) then no info, else showing info with number of seconds

::

::                                Elaboration:  Artur Zgadzaj        Status:  Free for use and distribute

setlocal

setlocal EnableExtensions

setlocal DisableDelayedExpansion

 

MD %TEMP% 2> nul

SET /A $Admin_Number=%RANDOM% * 100 / 32768 + 1

SET > "%TEMP%\$admin_%$Admin_Number%__SET.TXT"

 

SET "PATH_BAT=%~dp1"&if not exist "%~1" if not exist "%~1.*" SET "PATH_BAT="

 

 SET $Parameters=%*

setlocal EnableDelayedExpansion

 SET $Parameters=!$Parameters:%%=%%%%!

setlocal DisableDelayedExpansion

 

net session >nul 2>nul&if not errorlevel 1  goto Administrator_OK

 

SET "$Script=%PATH_BAT%%~nx1"

SET "$Script=%$Script:(=^(%"

SET "$Script=%$Script:)=^)%"

 

if defined ShowAdminInfo   (

   echo.

   echo Script = %$Script%

   echo.

   echo ******************************************************************************

   echo ***   R U N N I N G    A S    A D M I N I S T R A T O R    F O R   Y O U   ***

   echo ******************************************************************************

   echo.

   echo Call up just as the Administrator. You can make a shortcut to the script and set

   echo.

   echo          shortcut ^> Advanced ^> Running as Administrator

   echo.

   echo     Alternatively run once "As Administrator"

   echo     or in the Schedule tasks with highest privileges

   echo.

   echo Cancel Ctrl-C or wait for launch  %ShowAdminInfo%  seconds ...

   TIMEOUT /T %ShowAdminInfo% > nul

   )

 

SET "BatchFullName_EXE=%~1"&SET "EXT=%~x1"&SET "Start_EXE="

if /I not '%EXT%'=='.EXE'   SET "BatchFullName_EXE=%BatchFullName_EXE%.EXE"

if not defined $Admin_EXE  if exist "%BatchFullName_EXE%"  (SET Start_EXE=START "" /B) else (FOR /F %%I IN ("%BatchFullName_EXE%") DO (if not '%%~$PATH:I'==''  SET Start_EXE=START "" /B))

 

SET "Admin_Name=$admin_%$Admin_Number%"

SET "Inverted_Commas="

del "%TEMP%\%Admin_Name%_Start.bat" 2>nul

echo %$Parameters% > "%TEMP%\%Admin_Name%_Start.bat"

if not exist "%TEMP%\%Admin_Name%_Start.bat"  SET Inverted_Commas=^"

 

echo @echo off > "%TEMP%\%Admin_Name%_Start.bat"

echo setlocal DisableDelayedExpansion >> "%TEMP%\%Admin_Name%_Start.bat"

if not defined $Admin_Temp  echo SET TEMP^>^>"%TEMP%\%Admin_Name%__SET.TXT">> "%TEMP%\%Admin_Name%_Start.bat"

if not defined $Admin_SET   echo FOR /F ^"delims=^" %%%%A IN ^(%TEMP%\%Admin_Name%__SET.TXT^) DO SET %%%%A>> "%TEMP%\%Admin_Name%_Start.bat"

echo SET TMP=%%TEMP%%^&SET $Admin_Number=^&SET "PATH_BAT=%PATH_BAT%">> "%TEMP%\%Admin_Name%_Start.bat"

echo del "%TEMP%\%Admin_Name%__*.*" 2^>nul >> "%TEMP%\%Admin_Name%_Start.bat"

echo CD /D "%CD%" >> "%TEMP%\%Admin_Name%_Start.bat"

echo %Start_EXE% %$Parameters% %Inverted_Commas% >> "%TEMP%\%Admin_Name%_Start.bat"

 

echo SET UAC = CreateObject^("Shell.Application"^)                         > "%TEMP%\%Admin_Name%__getPrivileges.vbs"

echo UAC.ShellExecute "%TEMP%\%Admin_Name%_Start.bat", "", "", "runas", 1 >> "%TEMP%\%Admin_Name%__getPrivileges.vbs"

"%TEMP%\%Admin_Name%__getPrivileges.vbs"

endlocal

exit /B

 

:Administrator_OK

%$Parameters%

endlocal

goto:eof

REM *** A D M I N I S T R A T O R  - Automatically get admin rights  (The End)  ***