
SET SCRIPTS_DIR=%~dp0
SET SCRIPTS_DIR=%SCRIPTS_DIR:~0,-1%

call %SCRIPTS_DIR%\base_build.bat ace madara gams vrep vrep-config %*
