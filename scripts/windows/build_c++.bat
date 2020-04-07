
SET SCRIPTS_DIR=%~dp0
SET SCRIPTS_DIR=%SCRIPTS_DIR:~0,-1%

call %SCRIPTS_DIR%\base_build.bat madara gams forceboost forceosc vs2017 %*
