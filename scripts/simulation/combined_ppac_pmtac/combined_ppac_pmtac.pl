#!/usr/bin/perl
use lib "$ENV{GAMS_ROOT}/scripts/simulation";
use simulation;

$gams_root = $ENV{GAMS_ROOT};
$time = 3600;
$madara_debug = 0;
$gams_debug = 5;
$period = 0.5;
$num = 6;
$sim = "combined_ppac_pmtac";
$area = "small";
$border = "search_area.2";

simulation::run($num, $time, $period, $sim, $area, $madara_debug, $gams_debug, $border);
