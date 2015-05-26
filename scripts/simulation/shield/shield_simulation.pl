#!/usr/bin/perl
use lib "$ENV{GAMS_ROOT}/scripts/simulation";
use simulation;

$gams_root = $ENV{GAMS_ROOT};
$time = 3600;
$madara_debug = 0;
$period = 0.5;
$num = 10;
$sim = "shield";
$area = "small";

simulation::run($num, $time, $period, $sim, $area, $madara_debug);
