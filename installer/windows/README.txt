0. INTRODUCTION

This directory contains scripts to create a Nullsoft installer for GAMS
on Windows without users needing to download from repositories. Users
will still need to install perl and other prerequisites if they want
to generate projects.

1. HOW TO BUILD GAMS FOR THIS INSTALLER TO WORK

You must include the ace, madara, gams and tests features for this
installer script to run correctly. You can do so with the following
command in your Visual Studio Developer Command Prompt.

%GAMS_ROOT%\scripts\base_build ace madara gams tests vrep

Advanced users can build this the old fashioned way by enabling
tests and vrep, essentially.

2. HOW TO CREATE AN INSTALLER

  * Right-click on get_version.nsi and click Compile
  * Right-click on installer_*.nsi and click Compile. If prompted
    to allow get_version.exe to run, click Accept or Yes.