GAMS HELP DOCUMENTATION INFORMATION

0. INTRODUCTION

  VERSION: 1.0.0
  CONTACT: James Edmondson <jedmondson@gmail.com>
  WEBSITE: http://jredmondson.github.io/gams/

  This file provides general overviews about the help documentation in this
  directory and where to find additional help documentation where applicable.
  The other files in this directory are input for the Doxygen program to 
  generate the latest documentation for all of the GAMS tools.

1. OBTAINING PREREQUISITES

  Project: Doxygen (free)
  URL: http://www.doxygen.org

       Click on the downloads link and download the binaries/setup file for
       your operating system. It is unlikely that you will want to download
       the source. On many Linux systems, you can use the system package
       manager of your choice to download Doxygen. Most modern versions
       should work.

  Project: Graphviz (free)
  URL: http://www.graphviz.org

       Click on downloads and navigate to your operating system. Without this
       package, you will not be able to generate helpful documentation.

  Project: Java SDK including Javadocs (free)
  URL: Operating system dependent but generally you'll want the Sun JVM SDK
       
2. GENERATING DEVELOPER DOCUMENTATION

  When generating GAMS project files, provide docs=1 to generate documentation.
  
  cd $GAMS_ROOT
  mwc.pl -type gnuace -features docs=1 gams.mwc

  Can also use the base_build.bat|sh system with gams and docs options enabled

3 COMMITTING DEVELOPER DOCUMENTATION

  Developer documentation is not yet available online in its own branch.

