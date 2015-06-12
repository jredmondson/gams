#!/usr/bin/perl
use lib "$ENV{GAMS_ROOT}/scripts/simulation";
use simulation;

$gams_root = $ENV{GAMS_ROOT};
$time = 3600;
$madara_debug = 0;
$gams_debug = 5;
$period = 1;
$num = 6;
$sim = "min_time";
$area = "small";
$border = "search_area.1";
$num_coverages = 10;

simulation::run($num, $time, $period, $sim, $area, $madara_debug, $gams_debug, $border, $num_coverages);
