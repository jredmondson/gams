#!/usr/bin/perl
use lib "$ENV{GAMS_ROOT}/scripts/simulation";
use simulation;

$gams_root = $ENV{GAMS_ROOT};
$num = 10;
$time = 3600;
$period = 0.5;
$sim = "formation_large";
$area = "small";
$madara_debug = 0;
$gams_debug = 0;
$num_coverages = 0;
$launch_controllers = 1;

simulation::run($num, $time, $period, $sim, $area, $madara_debug, $gams_debug, $num_coverages, $launch_controllers);
