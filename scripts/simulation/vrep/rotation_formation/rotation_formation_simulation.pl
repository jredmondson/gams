#!/usr/bin/perl
use lib "$ENV{GAMS_ROOT}/scripts/simulation/vrep";
use simulation;

$num = 5;
$time = 3600;
$period = 0.5;
$sim = "rotation_formation";
$area = "small";
$madara_debug = 0;
$gams_debug = 6;
$border = "";
$num_coverages = 0;
$launch_controllers = 1;

simulation::run($num, $time, $period, $sim, $area, $madara_debug, $gams_debug, $border, $num_coverages, $launch_controllers);
