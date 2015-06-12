#!/usr/bin/perl
use lib "$ENV{GAMS_ROOT}/scripts/simulation";
use simulation;

$gams_root = $ENV{GAMS_ROOT};
$time = 3600;
$madara_debug = 0;
$gams_debug = 0;
$period = 0.75;
$num = 1;
$sim = "snake";
$area = "small";
$border = "region.0";

simulation::run($num, $time, $period, $sim, $area, $madara_debug, $gams_debug, $border);
