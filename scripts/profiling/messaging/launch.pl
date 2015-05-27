#!/usr/bin/perl
use lib "$ENV{GAMS_ROOT}/scripts/profiling";
use profiling;

#$gams_root = $ENV{GAMS_ROOT};
$scenario = "messaging";
$num = 2;
$time = 5;
$period = 0.5;
$debug = 5;

profiling::run($scenario, $num, $time, $period, $debug);
