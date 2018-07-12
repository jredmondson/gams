#!/bin/sh

#create g++ madara and gams debs
$GAMS_ROOT/scripts/linux/base_build.sh madara gams
$MADARA_ROOT/installer/debian/deb_installer_ia64.sh
$GAMS_ROOT/installer/debian/deb_installer_ia64.sh

$GAMS_ROOT/scripts/linux/base_build.sh madara python 
$MADARA_ROOT/installer/debian/deb_installer_ia64.sh --python

$GAMS_ROOT/scripts/linux/base_build.sh madara python ssl
$MADARA_ROOT/installer/debian/deb_installer_ia64.sh --python --ssl

#create g++ gams debs with ros2gams
$GAMS_ROOT/scripts/linux/base_build.sh gams ros
$GAMS_ROOT/installer/debian/deb_installer_ia64.sh --ros

#create g++ gams debs with ssl
$GAMS_ROOT/scripts/linux/base_build.sh madara gams ssl
$MADARA_ROOT/installer/debian/deb_installer_ia64.sh --ssl

$GAMS_ROOT/scripts/linux/base_build.sh madara zmq ssl
$MADARA_ROOT/installer/debian/deb_installer_ia64.sh --zmq --ssl

$GAMS_ROOT/scripts/linux/base_build.sh madara zmq python ssl
$MADARA_ROOT/installer/debian/deb_installer_ia64.sh --zmq --python --ssl

$GAMS_ROOT/scripts/linux/base_build.sh madara gams zmq ssl java
$MADARA_ROOT/installer/debian/deb_installer_ia64.sh --zmq --ssl --java
$GAMS_ROOT/installer/debian/deb_installer_ia64.sh --java

$GAMS_ROOT/scripts/linux/base_build.sh madara gams zmq python ssl java
$MADARA_ROOT/installer/debian/deb_installer_ia64.sh --zmq --python --ssl --java
$GAMS_ROOT/installer/debian/deb_installer_ia64.sh --java

#create g++ gams debs with ros2gams
$GAMS_ROOT/scripts/linux/base_build.sh madara gams ros java
$MADARA_ROOT/installer/debian/deb_installer_ia64.sh --java
$GAMS_ROOT/installer/debian/deb_installer_ia64.sh --ros --java

#create clang gams and madara debs
$GAMS_ROOT/scripts/linux/base_build.sh madara gams clang
$MADARA_ROOT/installer/debian/deb_installer_ia64.sh --clang
$GAMS_ROOT/installer/debian/deb_installer_ia64.sh --clang

$GAMS_ROOT/scripts/linux/base_build.sh madara gams clang python
$MADARA_ROOT/installer/debian/deb_installer_ia64.sh --clang python

$GAMS_ROOT/scripts/linux/base_build.sh madara zmq ssl clang python
$MADARA_ROOT/installer/debian/deb_installer_ia64.sh --clang --zmq --ssl --python

#create clang java gams and madara debs
$GAMS_ROOT/scripts/linux/base_build.sh madara gams java clang
$MADARA_ROOT/installer/debian/deb_installer_ia64.sh --clang --java
$GAMS_ROOT/installer/debian/deb_installer_ia64.sh --clang --java

$GAMS_ROOT/scripts/linux/base_build.sh madara zmq ssl java clang
$MADARA_ROOT/installer/debian/deb_installer_ia64.sh --clang --zmq --ssl --java

$GAMS_ROOT/scripts/linux/base_build.sh madara zmq ssl clang
$MADARA_ROOT/installer/debian/deb_installer_ia64.sh --clang --zmq --ssl


