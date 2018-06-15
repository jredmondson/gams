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

  When generating GAMS project files, you can do the following by hand.
  
  perl get_version.pl 
  doxygen Doxyfile_GAMS

  Can also use the base_build.bat|sh system with gams and docs options enabled

3 COMMITTING DEVELOPER DOCUMENTATION

  Developer documentation is not yet available online in its own branch.

4 TROUBLESHOOTING

  Windows:

    If you see errors related to the dot command, you likely do not have the
    Graphviz bin directory in your path. Graphviz is usually installed to
    C:\Program Files (x86)\Graphviz2.38, so simply add this to your PATH
    variable, restart Visual Studio, and everything should work.
