INSTRUCTIONS FOR SCRIPTS DIRECTORY


0. INTRODUCTION

This directory has a lot of old scripts that are no longer used. The
only script that we really use anymore is the base_build.sh script.
The base_build.sh script is useful for installing GAMS and all of its
prerequisites.


1. base_build.sh

The base_build.sh is coded to make downloading, building and updating
GAMS or its prerequisites as simple as possible.

1.1) HELP

  To get help, simply pass an invalid argument, such as 'help' to the script

1.2) BASIC DEBIAN PREREQUISITES

  For development with GAMS, you will need some basic prerequisites for
  building C++ applications and using the ACE build system. You can install
  these by passing "prereqs" to the base_build.sh, as in the following:
  
  base_build.sh prereqs
  
1.3) DEFAULT BUILD

  The default build for GAMS include ACE, MADARA, and C++ support. To install
  this very of GAMS, do the following (arguments can be in any order):
  
  base_build.sh prereqs ace madara gams tests
  
1.4) JAVA BUILD

  GAMS contains a Java port for most major features. To install base Java
  support, do the following (arguments can be in any order):
  
  base_build.sh prereqs ace madara gams tests java
  
1.5) ANDROID BUILD

  GAMS contains an Android port for most major features. Android is a sort
  of Java hybrid that can be used on the Google Android platform. To install 
  Android support, do the following (arguments can be in any order):
  
  base_build.sh prereqs ace madara gams tests android
  
1.6) NO TESTS

  If you do not want tests to be built, feel free to leave "tests" out of the 
  arguments to any of the previous base_build.sh examples.
  

  