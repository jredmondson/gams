INSTRUCTIONS FOR SCRIPTS DIRECTORY


0. INTRODUCTION

This directory contains scripts for building GAMS and its prerequisites
ACE and MADARA. 


1. base_build.bat

The base_build.bat is coded to make building GAMS or its prerequisites
as simple as possible.

1.1) HELP

  To get help, simply pass an invalid argument, such as 'help' to the script

1.2) PREREQUISITES

  For development with GAMS, you will need some basic prerequisites for
  building C++ applications and using the ACE build system. At the minimum,
  you will need to install Perl, SVN, Git, and VREP if you need a simulation
  environment.

  You will then need to set ACE_ROOT, MADARA_ROOT, GAMS_ROOT, and VREP_ROOT to
  their appropriate directory locations where you have downloaded the repos.

  Once you have the prerequisites installed, you need to open the Visual Studio
  development environment prompt which will allow these scripts to automate
  the process.

  ***************************************************************************
  YOU WILL NEED TO OPEN THE VISUAL STUDIO COMMAND PROMPT TO USE THESE SCRIPTS
  ***************************************************************************

  The Visual Studio command prompt is accessed by doing the following:

  Start->All programs->Visual Studio 20XX->Visual Studio Tools->Developer Command Prompt

  If you try to run these scripts from outside of this Developer Command Prompt,
  then they will not be able to refer to the msbuild system that allows for
  scripting Visual Studio builds and you will need to just compile the projects
  yourself.

  
  
1.3) DEFAULT BUILD

  The default build for GAMS includes ACE, MADARA, VREP, and C++ support with
  no tests, tutorials, or java support enabled. You can call the script 
  
  %GAMS_ROOT%\scripts\windows\build_c++.bat

  This is an alias for

  %GAMS_ROOT%\scripts\windows\base_build.bat ace madara gams vrep %*
  
1.4) JAVA BUILD

  GAMS contains a Java port for most major features. To install base Java
  support, do the following (arguments can be in any order):
  
  %GAMS_ROOT%\scripts\windows\build_c++.bat java

1.6) TESTS OR TUTORIALS

  If you would like tests and/or tutorials for MADARA and GAMS included
  then you will want to pass those as options to enable.
  
  %GAMS_ROOT%\scripts\windows\build_c++.bat tests tutorials

1.7) UPDATING GAMS, ACE, MADARA, etc.

  To rebuild all of the software, you can rerun build_c++ with the options
  you had originally specified (if any extra ones). To rebuild with the
  default C++ target options, you can run the following:

  %GAMS_ROOT%\scripts\windows\build_c++.bat

  However, you can also use the underlying base_build.bat script to target
  rebuilding only what is necessary. A general rule for selective rebuilding
  is that you must supply options for rebuilding anything that is based
  on the software you have updated. GAMS is built on MADARA and MADARA is
  built on ACE. If you have updated MADARA, then since GAMS depends on it,
  you will need to specify both GAMS and MADARA, because rebuilding MADARA
  will probably cause issues with a GAMS library that depended on it.

  The following are some examples of targeted rebuilds


  UPDATE MADARA AND GAMS WITH NO TESTS AND NO ACE REBUILD
  
  base_build.bat madara gams
  
  UPDATE MADARA AND GAMS WITH VREP SUPPORT
  
  base_build.bat madara gams vrep
  
  UPDATE MADARA ONLY WITH TESTS SUPPORT. THIS MAY CAUSE PROBLEMS WITH GAMS.
  
  base_build.bat madara tests
  
  
  

  