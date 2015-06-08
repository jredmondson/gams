#!/usr/bin/perl

my $gams_root = $ENV{"GAMS_ROOT"};
my $cmd = "\"$gams_root/gams_controller -i 0 -n 2 -p ros_p3dx --loop-time 60 --period 1 --madara-file $gams_root/scripts/stage_simulation/move/madara_init_0.mf\"";
system("xterm -hold -e $cmd &");
my $cmd = "\"$gams_root/gams_controller -i 1 -n 2 -p ros_p3dx --loop-time 60 --period 1 --madara-file $gams_root/scripts/stage_simulation/move/madara_init_1.mf\"";
system("xterm -hold -e $cmd &");
