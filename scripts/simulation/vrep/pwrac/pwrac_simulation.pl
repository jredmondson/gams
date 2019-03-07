#!/usr/bin/perl
use lib "$ENV{GAMS_ROOT}/scripts/simulation/vrep";
use simulation;

$gams_root = $ENV{GAMS_ROOT};
$time = 3600;
$madara_debug = 0;
$gams_debug = 0;
$period = 1;
$num = 6;
$sim = "pwrac";
$area = "small";
$border = "search_area.2";

simulation::run($num, $time, $period, $sim, $area, $madara_debug, $gams_debug, $border);
