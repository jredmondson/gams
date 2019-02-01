#!/usr/bin/perl
use lib "$ENV{GAMS_ROOT}/scripts/simulation";
use user_simulation;
use File::Basename;

# Create core variables for simulation
$dir = dirname($0);
$controller = "$ENV{GAMS_ROOT}/bin/gams_controller";
$duration = 300;
$madara_debug = 3;
$gams_debug = 3;
$period = 1;
$agents = 1;
@border = ();
$num_coverages = 0;
$launch_controllers = 1;
$domain = "gams_sims";
@hosts = ('239.255.0.1:4150');

# Rotate logs for comparisons
rename "$dir/agent_0.log", "$dir/agent_0.prev.log";

# Run simulation
user_simulation::run(
  controller => $controller,
  agents => $agents,
  duration => $duration,
  period => $period,
  dir => $dir,
  domain => $domain,
  madara_debug => $madara_debug,
  gams_debug => $gams_debug,
  multicast => \@hosts,
  border => \@border);
