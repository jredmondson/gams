#!/usr/bin/perl
use lib "$ENV{GAMS_ROOT}/scripts/simulation/vrep";
use simulation;

$gams_root = $ENV{GAMS_ROOT};
$time = 3600;
$period = 1;
$num = 4;
$sim = "formation_coverage";
$area = "small";
$madara_debug = 0;
$gams_debug = 6;
$border = "region.0";
$num_coverages = 0;
$launch_controllers = 1;

simulation::run($num, $time, $period, $sim, $area, $madara_debug, $gams_debug, $border, $num_coverages, $launch_controllers);
