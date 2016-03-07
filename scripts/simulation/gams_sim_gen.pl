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
my $vrep_start_port = 19906;
my $platform = 'vrep-quad';
my $multicast;
my $broadcast;
my @udp;

# setup options parser
GetOptions(
  'agents|a=i' => \$agents,
  'border=s' => \@borders,
  'broadcast|b=s' => \$broadcast,
  'duration|t=i' => \$duration,
  'force|f' => \$force,
  'gams_debug|gd=i' => \$gams_debug,
  'help|h' => \$help,
  'hz|z=i' => \$hz,
  'madara_debug|md=i' => \$madara_debug,
  'multicast|m=s' => \$multicast,
  'name|n=s' => \$name,
  'path|p=s' => \$path,
  'platform|l=s' => \$platform,
  'vrep_start_port|s' => \$vrep_start_port,
  'udp|u=s' => \@udp,
  'verbose|v' => \$verbose
  ) or $help = "yes";

#check for help request  
if ($help)
{
  my $output = "
$script purpose:

  Generates a new GAMS simulation

options:
  --agents|-a num        the number of agents
  --border name          name of region to draw border, can be multiple
  --broadcast|b host     broadcast ip/host to use for network transport
  --duration|-t sec      max duration of simulation in secs
  --force|-f             overwrite sim if already exists
  --gams_debug|-gd lev   log level for GAMS
  --help|-h              print guidance information
  --hz|-z hertz          periodic execution rate for agents
  --madara_debug|-md lev log level for MADARA
  --multicast|m host     multicast ip/host to use for network transport
  --name|-n name         sim name for directory creation
  --path|-p directory    directory path to create sim in
  --platform|-l model    GAMS platform model. Options are:
  
                         vrep-quad | vrep-uav : quadcopter
                         vrep-boat            : boat
                         vrep-ant             : ant robot
                         vrep-summit          : Summit robot
						
  --vrep_start_port|-s # VREP port number to start from
  --udp|u self h1 ...    udp ip/hosts to use for network transport
  --verbose|-v           print detailed debug info\n";
  
  print("$output\n");
}
else
{
  if (!$multicast and !$broadcast and scalar @udp == 0)
  {
    # if no transport, then specify default multicast
    $multicast = "239.255.0.1:4150";
  }

  if ($verbose)
  {
    my $output = "
$script will generate a sim with following params:
  agents = $agents
  border = " . (scalar @borders > 0 ?
    ("\n" . join ("\n    ", @borders)) : 'no') . "
  broadcast = " . ($broadcast ? $broadcast : 'no') . "
  duration = $duration
  force = " . ($force ? 'yes' : 'no') . "
  gams_debug = $gams_debug
  hz = $hz
  madara_debug = $madara_debug
  multicast = " . ($multicast ? $multicast : 'no') . "
  name = $name
  path = $path
  platform = $platform
  vrep_start_port = $vrep_start_port
  udp = " . (scalar @udp > 0 ? ("\n" . join ("\n    ", @udp)) : 'no') . "
";
	
    print("$output\n");
	
    print("Creating $path/$name...\n");
  }
  
  my $common_contents = "
/**
 * Common algorithm for all agents
 **/
.algorithm = 'null';

sensor.coverage.origin=[40.443077,-79.940570, 0.0];

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
.vrep_sw_position = [40.443077,-79.940570];
.vrep_ne_position = [40.443387,-79.940098];

/**
 * Surface can be concrete or water 
 **/
.surface = 'water';

/**
 * Define an example search area and region
**/
search_area.1.object_type = 4;
search_area.1.size = 1;
search_area.1.0 = 'region.0';

region.0.object_type = 1;
region.0.type = 0;
region.0.priority = 0;
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
  for (my $i = 0, my $port = $vrep_start_port; $i < $agents; ++$i, ++$port)
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
\$duration = $duration;
\$madara_debug = $madara_debug;
\$gams_debug = $gams_debug;
\$period = $period;
\$agents = $agents;
\@border = (" . join(",", @borders) . ");
\$num_coverages = 0;
\$launch_controllers = 1;
\@hosts = ";


  if ($broadcast)
  {
    $run_contents .= "('$broadcast');\n";
  }
  elsif (scalar @udp > 0)
  {
    $run_contents .= "('" . join ("', '", @udp) .  "');\n";
  }
  else
  {
    $run_contents .= "('$multicast');\n";
  }
  

  $run_contents .= "\n# Rotate logs for comparisons\n";

  # rotate logs
  for (my $i = 0; $i < $agents; ++$i)
  {
    $run_contents .= "rename \"\$dir/agent_$i.log\", ";
	  $run_contents .= "\"\$dir/agent_$i.prev.log\";\n";
  }

  $run_contents .= "
# Run simulation
user_simulation::run(
  agents => \$agents,
  duration => \$duration,
  period => \$period,
  dir => \$dir,
  madara_debug => \$madara_debug,
  gams_debug => \$gams_debug,\n";

  if ($broadcast)
  {
    $run_contents .= "  broadcast => \\\@hosts,";
  }
  elsif (scalar @udp > 0)
  {
    $run_contents .= "  udp => \\\@hosts,";
  }
  else
  {
    $run_contents .= "  multicast => \\\@hosts,";
  }
  
  $run_contents .= "
  border => \\\@border);\n";

  # create the run script
  open (my $run_file, '>', "$path/$name/run.pl");
    print $run_file $run_contents;
  close ($run_file);
  
}
