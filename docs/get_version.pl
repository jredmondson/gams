eval '(exit $?0)' && eval 'exec perl -w -S $0 ${1+"$@"}'
    & eval 'exec perl -w -S $0 $argv:q'
    if 0;
    
###################################################
# @author   James Edmondson <jedmondson@gmail.com>
# 
# This perl script is useful for generating a dynamic
# project number to be used by Doxygen in its help file
# generation process.
###################################################
    
open VERSION_FILE, "<",  $ENV{'GAMS_ROOT'} . "/VERSION.txt" or
    die "Unable to open ../VERSION.txt for reading.";
  $version = <VERSION_FILE>;
close VERSION_FILE;

open OUTPUT_FILE, ">",  $ENV{'GAMS_ROOT'} . "/docs/VERSION.txt" or
    die "Unable to open VERSION.txt for writing.";
  print OUTPUT_FILE  "PROJECT_NUMBER = " . $version . "\n";
close OUTPUT_FILE;

