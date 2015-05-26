#!/usr/bin/perl
use lib "$ENV{GAMS_ROOT}/scripts/simulation";
use simulation;

$gams_root = $ENV{GAMS_ROOT};
$time = 3600;
$madara_debug = 0;
$period = 1;
$num = 10;
$sim = "prioritized_min_time";
$area = "small";
$plants = "search_area.2";

simulation::run($num, $time, $period, $sim, $area, $madara_debug, $plants);
