#!/usr/bin/perl

###################################################
# @brief The GAMS Simulation Generator for VREP
# @author James Edmondson <jedmondson@gmail.com>
# @date 2016
###################################################

use strict;
use warnings;
use Getopt::Long qw(GetOptions);
use File::Path qw(make_path);
use File::Basename;

my $script = fileparse($0);
my $agents = "1";
my $path = ".";
my $name = "simulation";
my $hz = "1";
my $duration = 300;
my $madara_debug = 3;
my $gams_debug = 3;
my @borders = ();
my $help;
my $verbose;
my $force;
my $starting_port = 19906;
my $platform = 'vrep-quad';

# setup options parser
GetOptions(
  'agents|a=i' => \$agents,
  'border|b=s' => \@borders,
  'duration|t=i' => \$duration,
  'force|f' => \$force,
  'gams_debug|g=i' => \$gams_debug,
  'help|h' => \$help,
  'hz|z=i' => \$hz,
  'madara_debug|m=i' => \$madara_debug,
  'name|n=s' => \$name,
  'path|p=s' => \$path,
  'platform|l=s' => \$platform,
  'starting_port|s' => \$starting_port,
  'verbose|v' => \$verbose
  ) or $help = "yes";

#check for help request  
if ($help)
{
  my $output = "
$script purpose:

  Generates a new GAMS simulation

options:
  --agents|-a num       the number of agents
  --border|-b name      name of region to draw border
  --duration|-t sec     max duration of simulation in secs
  --force|-f            overwrite sim if already exists
  --gams_debug|-g level log level for GAMS
  --help|-h             print guidance information
  --hz|-z hertz         periodic execution rate for agents
  --madara_debug|-m lev log level for MADARA
  --name|-n name        sim name for directory creation
  --path|-p directory   directory path to create sim in
  --platform|-l model   GAMS platform model. Options are:
  
                        vrep-quad | vrep-uav : quadcopter
                        vrep-boat            : boat
                        vrep-ant             : ant robot
                        vrep-summit          : Summit robot
						
  --starting_port|-s #  VREP port number to start from
  --verbose|-v          print detailed debug info\n";
  
  print("$output\n");
}
else
{
  if ($verbose)
  {
    my $output = "
$script will generate a sim with following params:
  agents = $agents
  border = @borders
  duration = $duration
  gams_debug = $gams_debug
  hz = $hz
  madara_debug = $madara_debug
  name = $name
  path = $path
  platform = $platform
  starting_port = $starting_port\n";
	
	print("$output\n");
	
	print("Creating $path/$name...\n");
  }
  
  my $common_contents = "
/**
 * Common algorithm for all agents
 **/
.algorithm = 'null';

/**
 * Type of platform to use. Options include:
 * vrep-quad       : A VREP quadcopter
 * vrep-quad-laser : A VREP quadcopter
 * vrep-boat       : A VREP boat
 * vrep-ant        : A VREP ant-like ground robot
 * vrep-summit     : A VREP Summit robot
 * 
 * Specialty options (must be compiled with more than just vrep feature)
 * ros-p3dx        : A ROS Pioneer 3DX robot
 **/
.platform = '$platform';

/**
 * The host information where VREP is running
 **/
.vrep_host = '127.0.0.1';

/**
 * Max distance (meters) to move target when VREPBase::move is invoked,
 * if no move thread is being used (vrep_move_thread_rate = 0)
 **/
.vrep_max_delta = 1;

/**
 * Hertz rate to run VREPBase move thread. Set to zero to disable thread
 **/
.vrep_move_thread_rate = 10;

/**
 * Each thread tick, target will move at most vrep_thread_move_speed divided
 * by vrep_move_thread_rate meters.
 **/
.vrep_thread_move_speed = 2;";

  
  my $env_contents = "
/**
 * Set VREP view area GPS mapping
**/
.vrep_sw_position = '40.443077,-79.940570';
.vrep_ne_position = '40.443387,-79.940098';
.surface = 'water';

/**
 * Define an example search area and region
**/
search_area.1.object_type = 4;
search_area.1.size = 1;
search_area.1.0 = 'region.0';

region.0.object_type = 1;
region.0.type = 0;
region.0.size = 4;
region.0.0 = [40.443237, -79.940570];
region.0.1 = [40.443387, -79.940270];
region.0.2 = [40.443187, -79.940098];
region.0.3 = [40.443077, -79.940398];\n";

  
  # recursively create path/name
  my $create_result = make_path("$path/$name");
  
  if (!$force)
  {
    $create_result or die "ERROR: Unable to create simulation directory\n";
  }
  
  if ($verbose)
  {
	print("Creating area environment file...\n");
  }
  
  # create the environment file
  open (my $env_file, '>', "$path/$name/env.mf");
    print $env_file $env_contents;
  close ($env_file);
  
  if ($verbose)
  {
	print("Creating common initialization file...\n");
  }
  
  # create the common file
  open (my $common_file, '>', "$path/$name/common.mf");
    print $common_file $common_contents;
  close ($common_file);
  
  
  if ($verbose)
  {
	print("Creating individual agent initialization files...\n");
  }
  
  # create the individual agent initialization
  for (my $i = 0, my $port = $starting_port; $i < $agents; ++$i, ++$port)
  {
    my $agent_contents = "
// Setup VREP port for agent
.vrep_port = $port;

/**
 * Change the following to unique coordinates
 * for each agent file
 **/
.initial_lat = 40.443136;
.initial_lon = -79.940274;
.initial_alt = 4;

/**
 * Set to .algorithm variable from common.mf
 * Feel free to change this to whatever you like
 **/
agent.$i.command = .algorithm;\n";
	
	open (my $agent_file, '>', "$path/$name/agent_$i.mf");
	  print $agent_file $agent_contents;
	close ($agent_file);
  }
  
  if ($verbose)
  {
	print("Creating simulation perl script...\n");
  }
  
  my $period = 1 / $hz;
  
  my $run_contents = "#!/usr/bin/perl
use lib \"\$ENV{GAMS_ROOT}/scripts/simulation\";
use user_simulation;
use File::Basename;

# Create core variables for simulation
\$dir = dirname(\$0);
\$time = $duration;
\$madara_debug = $madara_debug;
\$gams_debug = $gams_debug;
\$period = $period;
\$num = $agents;
\$border = (" . join(",", @borders) . ");
\$num_coverages = 0;
\$launch_controllers = 1;

# Rotate logs for comparisons\n";

  # rotate logs
  for (my $i = 0; $i < $agents; ++$i)
  {
    $run_contents .= "rename \"\$dir/agent_$i.log\", ";
	$run_contents .= "\"\$dir/agent_$i.prev.log\";\n";
  }

  $run_contents .= "
# Run simulation
user_simulation::run(\$num, \$time, \$period,
  \$dir, \$madara_debug, \$gams_debug, 
  \$border, \$num_coverages, \$launch_controllers);\n";

  # create the run script
  open (my $run_file, '>', "$path/$name/run.pl");
    print $run_file $run_contents;
  close ($run_file);
  
}
