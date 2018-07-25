#!/usr/bin/perl

###################################################
# @brief The GAMS Project Configurator
# @author James Edmondson <jedmondson@gmail.com>
# @date 2016
###################################################

use strict;
use warnings;
use Getopt::Long qw(GetOptions);
use File::Path qw(make_path);
use File::Basename;
use File::Copy qw(copy);
use POSIX;
use Scalar::Util qw(looks_like_number);


my ($script, $script_dir) = fileparse($0);
my $algorithm;
my $agents;
my @border = ();
my $broadcast;
my $domain;
my $duration;
my $equidistant;
my $buffer = 5;
my $first;
my $gams_debug;
my $group;
my $help;
my $hz;
my $invert;
my $last;
my $location;
my $madara_debug;
my $max_lat;
my $max_lon;
my $min_height;
my $min_lat;
my $min_lon;
my $multicast;
my $height_diff = 1;
my @new_algorithm;
my @new_platform;
my @new_platform_thread;
my @new_thread;
my @new_transport;
my @new_receive_filters;
my @new_send_filters;
my $ordered;
my $path = '.';
my $permute;
my $platform;
my @points;
my $priority;
my $randomize;
my $range_specified;
my $region;
my $remove_files;
my $remove_logs;
my $rotate;
my $surface;
my $thread_hz;
my @udp;
my $unique;
my $update_vrep;
my $verbose;
my $vrep_start_port;
my $sim_path;
my $src_path;
my $algs_path;
my $algs_threads_path;
my $containers_path;
my $docs_path;
my $plats_path;
my $plats_threads_path;
my $threads_path;
my $trans_path;
my $filters_path;
my $transient;
my $bin_path;
my @algorithms = ();
my @algorithm_threads = ();
my @containers = ();
my @platforms = ();
my @platform_threads = ();
my @threads = ();
my @transports = ();
my @filters = ();
my $gams_root = $ENV{"GAMS_ROOT"};
my $algs_templates = "$gams_root/scripts/simulation/templates/algorithms";

$algs_templates =~ s/\\/\//g;

# setup options parser
GetOptions(
  'algorithm|alg=s' => \$algorithm,
  'agents|a=i' => \$agents,
  'buffer=i' => \$buffer,
  'border=s' => \@border,
  'broadcast|b=s' => \$broadcast,
  'container|c=s' => \@containers,
  'domain=s' => \$domain,
  'duration|d|t=i' => \$duration,
  'equidistant|distributed' => \$equidistant,
  'first|f=i' => \$first,
  'gams-debug|gams_debug|gd=i' => \$gams_debug,
  'group=s' => \$group,
  'help|h' => \$help,
  'height_diff|height-diff=i' => \$height_diff,
  'hz|z=i' => \$hz,
  'invert|i' => \$invert,
  'last|l=i' => \$last,
  'location|position=s' => \$location,
  'madara-debug|madara_debug|md=i' => \$madara_debug,
  'max_lat|max-lat=f' => \$max_lat,
  'max_lon|max-lon=f' => \$max_lon,
  'min_height|min-height|e=i' => \$min_height,
  'min_lat|min-lat=f' => \$min_lat,
  'min_lon|min-lon=f' => \$min_lon,
  'multicast|m=s' => \$multicast,
  'new-algorithm|new_algorithm|new_alg|na=s' => \@new_algorithm,
  'new-platform|new_platform|new_plat|np=s' => \@new_platform,
  'new-platform-thread|new_platform_thread|npt=s' => \@new_platform_thread,
  'on-receive|receive=s' => \@new_receive_filters,
  'on-send|send=s' => \@new_send_filters,
  'new-thread|new_thread|nt=s' => \@new_thread,
  'new-transport|new_transport|nr=s' => \@new_transport,
  'ordered|o' => \$ordered,
  'path|dir|p=s' => \$path,
  'permute|u=s' => \$permute,
  'platform|l=s' => \$platform,
  'priority=s' => \$priority,
  'randomize' => \$randomize,
  'region|r=s' => \$region,
  'remove_files|remove-files' => \$remove_files,
  'remove_logs|remove-logs' => \$remove_logs,
  'rotate|t' => \$rotate,
  'surface|s=s' => \$surface,
  'thread_hz|thread-hz|thz=f' => \$thread_hz,
  'transient' => \$transient,
  'vrep-start-port|vrep_start_port|s=i' => \$vrep_start_port,
  'unique' => \$unique,
  'udp|u=s' => \@udp,
  'update-vrep|update_vrep|vrep' => \$update_vrep,
  'verbose|v' => \$verbose
  ) or $help = "yes";

#check for help request  
if ($help)
{
  my $output = " 
$script purpose:

  Configures GAMS agent settings for a simulation

options:
  --algorithm|-alg alg   configures agent files to use a specific algorithm
  --agents|-a num        number of agents that need to be in simulation
  --buffer|-m meters     buffer in meters between agents
  --border r0 r1 ...     regions to put a border around
  --broadcast|b host     broadcast ip/host to use for network transport
  --container|c name     container data structure to modify
  --domain domain        domain of the network (to separate network traffic)
  --distributed          for positions in region, distribute uniformly
  --duration|-t sec      max duration of simulation in secs
  --equidistant          alias to --distributed
  --first|-f num         first agent number (e.g 0, 1, 2, etc.)
  --gams-debug|-gd lev   log level for GAMS
  --group  prefix        group to change
  --help|-h              print guidance information
  --height-diff meters   height difference in meters when paired with unique
  --hz|-z hertz          periodic execution rate for agents
  --invert|-i            invert the x and y axis in a formation
  --last|-l num          last agent number (e.g. 1, 2, 3, etc.)
  --location|--position p specifies a specific position in the format [x, y],
                         where x and y should be doubles
  --madara-debug|-md lev log level for MADARA
  --max-lat|max-lat x    defines the maximum latitude, generally for a region
  --max-lon|max-lon x    defines the maximum longitude, generally for a region
  --min-height|-e num    height in meters
  --min-lat|min-lat x    defines the minimum latitude, generally for a region
  --min-lon|min-lon x    defines the minimum longitude, generally for a region
  --multicast|m host     multicast ip/host to use for network transport
  --new-algorithm|na name create infrastructure for custom algorithm
  --new-platform|np name create infrastructure for custom platform
  --new-platform-thread|ntp name 
                         create infrastructure for a custom platform thread
  --new-thread|nt name   create infrastructure for custom thread
  --new-transport|nr name create infrastructure for custom network transport
  --on-send|send name    create an on-send filter 
  --on-receive|receive name  create an on-receive filter 
  --ordered              order distribution by agent id l->r, t->b
  --path|-p|--dir dir    the directory path to a simulation
  --permute|-u           permute existing locations or heights 
  --platform|-l model    GAMS platform model. Options are:
  
                         vrep-quad | vrep-uav : quadcopter
                         vrep-boat            : boat
                         vrep-ant             : ant robot
                         vrep-summit          : Summit robot
            
  --priority level       define the priority, generally to be used for regions
  --randomize            randomize the target locations or heights  
  --remove-files         when removing agents, remove conf files 
  --remove-logs          when removing agents, remove log files 
  --rotate|-t            rotate a formation 
  --region|-r region     region name for starting locations   
  --surface|-s type      change the surface to type. Options are:   
  
                         concrete   : generic land model
                         water      : water model (e.g., for boats)
                         
  --thread-hz|-thz hz    the hertz to run threads at  
  --transient            indicates a transient group should be created/used
  --udp|u self h1 ...    udp ip/hosts to use for network transport 
  --unique|-u            requires unique attribute (e.g., height)
  --update-vrep|vrep     updates the VREP bounding box with a min|max-lat|lon
  --verbose|-v           print detailed debug info
  --vrep-start-port|-s # VREP port number to start from\n";
  
  print("$output\n");
}
else
{
  if ($verbose)
  {
    my $output = "
$script is using the following configuration:
  algorithm = " . ($algorithm ? $algorithm : 'no change') . "
  agents = " . ($agents ? $agents : 'no change') . "
  border = " . (scalar @border > 0 ?
    ("\n    " . join ("\n    ", @border)) : 'no') . "
  broadcast = " . ($broadcast ? $broadcast : 'no') . "
  buffer = $buffer meters
  containers = " . (scalar @containers > 0 ?
    ("\n    " . join ("\n    ", @containers)) : 'no') . "
  domain = " . (defined $domain ? $domain : 'default') . "
  equidistant = " . ($equidistant ? 'yes' : 'no') . "
  first = " . (defined $first ? $first : 'default') . "
  gams_debug = " . ($gams_debug ? $gams_debug : 'no change') . "
  group = " . ($group ? $group : 'none specified') . "
  height_diff = $height_diff
  hz = " . ($hz ? $hz : 'no change') . "
  invert = " . ($invert ? 'yes' : 'no') . "
  last = " . (defined $last ? $last : 'default') . "
  location = " . ($location ? $location : 'no change') . "
  madara_debug = " . ($madara_debug ? $madara_debug : 'no change') . "
  max_lat = " . ($max_lat ? $max_lat : 'no change') . "
  max_lon = " . ($max_lon ? $max_lon : 'no change') . "
  min_height = " . ($min_height ? $min_height : 'no change') . "
  min_lat = " . ($min_lat ? $min_lat : 'no change') . "
  min_lon = " . ($min_lon ? $min_lon : 'no change') . "
  multicast = " . ($multicast ? $multicast : 'no') . "
  new_algorithm = " . (scalar @new_algorithm > 0 ?
    ("\n    " . join ("\n    ", @new_algorithm)) : 'no') . "
  new_platform = " . (scalar @new_platform > 0 ?
    ("\n    " . join ("\n    ", @new_platform)) : 'no') . "
  new_platform_thread = " . (scalar @new_platform_thread > 0 ?
    ("\n    " . join ("\n    ", @new_platform_thread)) : 'no') . "
  new_thread = " . (scalar @new_thread > 0 ?
    ("\n    " . join ("\n    ", @new_thread)) : 'no') . "
  new_transport = " . (scalar @new_transport > 0 ?
    ("\n    " . join ("\n    ", @new_transport)) : 'no') . "
  on_receive = " . (scalar @new_receive_filters > 0 ?
    ("\n    " . join ("\n    ", @new_receive_filters)) : 'no') . "
  on_send = " . (scalar @new_send_filters > 0 ?
    ("\n    " . join ("\n    ", @new_send_filters)) : 'no') . "
  ordered = " . ($ordered ? 'yes' : 'no') . "
  path = $path
  platform = " . ($platform ? $platform : 'no change') . "
  priority = " . ($priority ? $priority : 'no change') . "
  randomize = " . ($randomize ? 'yes' : 'no') . "
  remove_files = " . ($remove_files ? 'yes' : 'no') . "
  remove_logs = " . ($remove_logs ? 'yes' : 'no') . "
  rotate = " . ($rotate ? 'yes' : 'no') . "
  region = " . ($region ? $region : 'none specified') . "
  surface = " . ($surface ? $surface : 'none specified') . "
  thread_hz = " . ($thread_hz ? $thread_hz : 'none specified') . "
  transient = " . ($transient ? 'yes' : 'no') . "
  vrep_start_port = " . ($vrep_start_port ? $vrep_start_port : 'no change') . "
  udp = " . (scalar @udp > 0 ? ("\n    " . join ("\n    ", @udp)) : 'no') . "
  unique = " . ($unique ? 'yes' : 'no') . "\n";
  
    print("$output\n");
  }

  # recursively create path/name
  my $create_result = make_path("$path");
  
  $docs_path = "$path/docs";
  $sim_path = "$path/sim";
  $src_path = "$path/src";
  $algs_path = "$path/src/algorithms";
  $algs_threads_path = "$path/src/algorithms/threads";
  $containers_path = "$path/src/containers";
  $plats_path = "$path/src/platforms";
  $plats_threads_path = "$path/src/platforms/threads";
  $threads_path = "$path/src/threads";
  $trans_path = "$path/src/transports";
  $filters_path = "$path/src/filters";
  $bin_path = "$path/bin";
  
  if ($create_result)
  {
    $algorithm = $algorithm ? $algorithm : "null";
    $agents = $agents ? $agents : 1;
    $hz = $hz ? $hz : 1;
    $domain = $domain ? $domain : "gams_sims";
    $duration = $duration ? $duration : 300;
    $madara_debug = $madara_debug ? $madara_debug : 3;
    $gams_debug = $gams_debug ? $gams_debug : 3;
    $vrep_start_port = $vrep_start_port ? $vrep_start_port : 19906;
    $platform = $platform ? $platform : 'vrep-quad';
    $multicast = $multicast ? $multicast : "239.255.0.1:4150";
  
    if ($verbose)
    {
      print("Directory $path had not been created. Populating...\n");
    }
    
    make_path("$docs_path");
    make_path("$sim_path");
    make_path("$algs_path");
    make_path("$algs_threads_path");
    make_path("$containers_path");
    make_path("$plats_path");
    make_path("$plats_threads_path");
    make_path("$threads_path");
    make_path("$trans_path");
    make_path("$filters_path");
    make_path("$bin_path");
    
  my $common_contents = "
/**
 * Common algorithm that may be used to initialize agent algorithms
 **/
.algorithm = '$algorithm';

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

sensor.coverage.origin=[40.443077,-79.940570, 0.0];

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
.vrep_thread_move_speed = 2;\n";

  
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
 * Define an example search area. Search areas can contain multiple regions.
**/
search_area.1.object_type = 4;
search_area.1.size = 1;
search_area.1.0 = 'region.0';

/**
 * Define region.0
**/
region.0.object_type = 1;
region.0.type = 0;
region.0.priority = 0;
region.0.size = 4;
region.0.0 = [40.443237, -79.940570];
region.0.1 = [40.443387, -79.940270];
region.0.2 = [40.443187, -79.940098];
region.0.3 = [40.443077, -79.940398];\n";

    if ($verbose)
    {
      print("Creating area environment file...\n");
    }
    
    # create the environment file
    open (env_file, '>', "$sim_path/env.mf");
      print env_file $env_contents;
    close (env_file);
    
    if ($verbose)
    {
      print("Creating common initialization file...\n");
    }
    
    # create the common file
    open (common_file, '>', "$sim_path/common.mf");
      print common_file $common_contents;
    close (common_file);
    
    
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
 * Set the algorithm for the agent
 **/
agent.$i.algorithm = .algorithm;\n";
    
      open (agent_file, '>', "$sim_path/agent_$i.mf");
        print agent_file $agent_contents;
      close (agent_file);
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
\$controller = \"\$ENV{GAMS_ROOT}/bin/gams_controller\";
\$duration = $duration;
\$madara_debug = $madara_debug;
\$gams_debug = $gams_debug;
\$period = $period;
\$agents = $agents;
\@border = (" . join(",", @border) . ");
\$num_coverages = 0;
\$launch_controllers = 1;
\$domain = \"$domain\";
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
  controller => \$controller,
  agents => \$agents,
  duration => \$duration,
  period => \$period,
  dir => \$dir,
  domain => \$domain,
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
    open (run_file, '>', "$sim_path/run.pl");
      print run_file $run_contents;
    close (run_file);
  } # end create directory
  else
  {
    if ($verbose)
    {
      print("Directory $path already existed. Not creating files.\n");
    }
  }
  
  if (!$create_result and $agents)
  {
    # The user has requested a change to the number of agents
    
    if ($verbose)
    {
      print ("Reading agents info from $sim_path/run.pl\n");
    }
    
    my $run_contents;
    open run_file, "$sim_path/run.pl" or
      die "ERROR: Couldn't open $sim_path/run.pl\n"; 
      $run_contents = join("", <run_file>); 
    close run_file;
    
    if ($run_contents =~ m{agents\s*=\s*(\d+)})
    {
      my $old_agents = $1;
      
      if ($verbose)
      {
        print ("  Old agent number is $old_agents\n");
        print ("  New agent number is $agents\n");
      }
    
      if ($old_agents < $agents)
      {
        # we have to add more agent files. The file for a new agent
        # is pretty simple, but to make this smart, we need to open
        # the first agent's file and read in the vrep port info. The
        # new agents will all have an appropriate offset
        
        my $agent_contents;
        my $first_port;
        if (open run_file, "$sim_path/agent_0.mf")
        {
          $agent_contents = join("", <run_file>); 
          close run_file;
          
          # we check for vrep port info and set the variable appropriately
          if ($agent_contents =~ m{\.vrep_port\s*=\s*(\d+)})
          {
            $first_port = $1;
            if ($verbose)
            {
              print ("  Found vrep_port. Starting at $first_port\n");
            }
          }
          else
          {
            # The user appears to have altered the first agent file
          
            $first_port = 19906;
            if ($verbose)
            {
              print ("  Could not find vrep_port. Starting at $first_port\n");
            }
          }
          
        }
        else
        {
          # The user appears to have deleted all agents files. So,
          # create a new agent file from scratch using reasonable defaults
          
          $first_port = 19906;
          
          
          if ($verbose)
          {
            print ("  Could not read agent.0.mf. Starting at $first_port\n");
          }
        }
        
        if ($verbose)
        {
          print ("  Creating agent $old_agents through " . ($agents - 1) . "...\n");
        }
    
        my $port = $first_port + $old_agents;
        for (my $i = $old_agents; $i < $agents; ++$i, ++$port)
        {
          $agent_contents = "
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
agent.$i.algorithm = .algorithm;\n";

	        if (open (my $agent_file, '>', "$sim_path/agent_$i.mf"))
          {
	          print $agent_file $agent_contents;
	          close ($agent_file);
          }
          else
          {
            print ("ERROR: Unable to write to file $sim_path/agent_$i.mf");
          }
        }
      }
      elsif ($old_agents > $agents)
      {
        # delete agent files and logs if requested
        for (my $i = $agents; $i < $old_agents; ++$i)
        {
          if ($remove_files)
          {
            if ($verbose)
            {
              print ("  Deleting $sim_path/agent_$i.mf...\n");
            }
    
            unlink "$sim_path/agent_$i.mf";
          }
          if ($remove_logs)
          {
            if ($verbose)
            {
              print ("  Deleting $sim_path/agent_$i.*.log...\n");
            }
    
            unlink "$sim_path/agent_$i.log";
            unlink "$sim_path/agent_$i.prev.log";
          }
        }
      }

      $run_contents =~ s/(agents\s*=\s*)\d+/$1$agents/;
      
      my $log_rotate_commands = "";
      # rotate logs
      for (my $i = 0; $i < $agents; ++$i)
      {
        $log_rotate_commands .= "rename \"\$dir/agent_$i.log\", ";
        $log_rotate_commands .= "\"\$dir/agent_$i.prev.log\";\n";
      }

      $run_contents =~ s/Rotate logs for comparisons(.|\s)*# Run simulation/Rotate logs for comparisons\n${log_rotate_commands}\n# Run simulation/;
      
      open run_file, ">$sim_path/run.pl" or
        die "ERROR: Couldn't open $sim_path/run.pl for writing\n";
        print run_file  $run_contents; 
      close run_file;
          
    }
    else
    {
      print ("ERROR: Couldn't find agents info in $sim_path/run.pl\n");
    }
  }  
  
  # some operations need a hint on whether or not the user specified a range
  if (defined $first or defined $last)
  {
    $range_specified = 1;
  }

  # if either first or last has not been specified, create an agent range.  
  if (not defined $first or not defined $last)
  {
    # if we have not defined agents, read the agent number or use default of 1
    if (!$agents)
    {
      my $run_contents;
      open run_file, "$sim_path/run.pl" or
        die "ERROR: Couldn't open $sim_path/run.pl\n"; 
        $run_contents = join("", <run_file>); 
      close run_file;
      
      if ($run_contents =~ m{agents\s*=\s*(\d+)})
      {
        $agents = $1;
      }
      else
      {
        $agents = 1;
      }
    }
    
    # default first is the first agent (0)
    if (not defined $first)
    {
      $first = 0;
    }
    
    # default last is $agents - 1
    if (not defined $last)
    {
      $last = $agents - 1;
    }
    
    if ($verbose)
    {
      print ("Defaults init: first = $first, last = $last, agents=$agents\n");
    }
  }
  else
  {
    # if we have not defined agents, read the agent number or use default of 1
    if (!$agents)
    {
      my $run_contents;
      open run_file, "$sim_path/run.pl" or
        die "ERROR: Couldn't open $sim_path/run.pl\n"; 
        $run_contents = join("", <run_file>); 
      close run_file;
      
      if ($run_contents =~ m{agents\s*=\s*(\d+)})
      {
        $agents = $1;
      }
      else
      {
        $agents = 1;
      }
    }
    
    if ($verbose)
    {
      print ("User-defined init: first = $first, last = $last, agents=$agents\n");
    }
  }

  if ($location)
  {
    if ($verbose)
    {
      print ("Changing location of agent $first to $last to $location\n");
    }
    
    my ($lat, $lon);
    
    if ($location =~ m{(-?\d*\.\d*)+\s*,\s*(-\d*\.\d*)+})
    {
      $lat = $1;
      $lon = $2;
      
      # Try to replace all algorithm arguments and comments
      for (my $i = $first; $i <= $last; ++$i)
      {
        # we have a valid point. Change the agent's init file.
        my $agent_contents;
        open agent_file, "$sim_path/agent_$i.mf" or
          die "ERROR: Couldn't open $sim_path/agent_$i.mf for reading\n";
          $agent_contents = join("", <agent_file>); 
        close agent_file;
        
        # replace lat and lon with our new points
        $agent_contents =~ s/(\.initial_lat\s*=\s*)\-?\d+\.\d+/$1$lat/;
        $agent_contents =~ s/(\.initial_lon\s*=\s*)\-?\d+\.\d+/$1$lon/;
           
        open agent_file, ">$sim_path/agent_$i.mf" or
          die "ERROR: Couldn't open $sim_path/agent_$i.mf for writing\n";
          print agent_file  $agent_contents; 
        close agent_file;
      
      }
    }
  }  
  
  if ($algorithm)
  {
    if ($verbose)
    {
      print ("Changing algorithm to $algorithm for agent[$first, $last]\n");
    }
    
    my $alg_dir = $algorithm;
    
    if ($verbose)
    {
      print ("  Checking for $algs_templates/$alg_dir directory\n");
    }
    
    if (not -d "$algs_templates/$alg_dir")
    {
      if ($verbose)
      {
        print ("  Dir $algs_templates/$alg_dir does not exist\n");
      }
    
      my $aliases_contents;
      
      if ($verbose)
      {
        print ("  Opening $algs_templates/aliases.conf\n");
      }
    
      open aliases_file, "$algs_templates/aliases.conf" or
        die "ERROR: Couldn't open $algs_templates/aliases.conf\n"; 
        $aliases_contents = join("", <aliases_file>); 
      close aliases_file;
      
      if ($verbose)
      {
        print ("  Looking for $algorithm in aliases contents\n");
      }
    
      if ($aliases_contents =~ m{$algorithm\s*=\s*(.+)})
      {
        if ($verbose)
        {
          print ("    $algorithm is aliased to dir $1\n");
        }
    
        $alg_dir = $1;
      }
      else
      {
        if ($verbose)
        {
          print ("    $algorithm does not have a directory\n");
        }
    
        $alg_dir = undef;
      }
    }
    else
    {
      $alg_dir = $algorithm;
    }
    
    my $replacement = "agent.0.algorithm = \"$algorithm\";\n\n";
      
    if ($alg_dir)
    {
      if ($verbose)
      {
        print ("  Opening $algs_templates/$alg_dir/args.conf\n");
      }
    
      my @args_list;
      
      if (open args_file, "$algs_templates/$alg_dir/args.conf")
      {
        if ($verbose)
        {
          print ("  Reading $algs_templates/$alg_dir/args.conf\n");
        }
    
        @args_list = <args_file>; 
        close args_file;
        
        # iterate over the args list to find args
        for my $line (@args_list)
        {
          if ($verbose)
          {
            print ("  Parsing $line\n");
          }
          
          # if this contains an actual arg variable = value
          if ($line =~ m{([^\s]+)\s*=\s*(.*)})
          {
            my ($key, $value) = ($1, $2);
            $replacement .= "agent.0.algorithm.args.$key = $value\n";
            
            if ($verbose)
            {
              print ("  Parsed agent.0.algorithm.args.$key = $value\n");
            }
          }
          else
          {
            if ($verbose)
            {
              print ("  Parsed comment or blank line\n");
            }
            $replacement .= $line;
          }
        }
      } # end if open args_file
      else
      {
        if ($verbose)
        {
          print ("  Unable to read $algs_templates/$alg_dir/args.conf\n");
        }
      }
    }
    
    # Try to replace all algorithm arguments and comments
    for (my $i = $first; $i <= $last; ++$i)
    {
      # we have a valid point. Change the agent's init file.
      my $agent_contents;
      open agent_file, "$sim_path/agent_$i.mf" or
        die "ERROR: Couldn't open $sim_path/agent_$i.mf for reading\n";
        $agent_contents = join("", <agent_file>); 
      close agent_file;
     
      if ($verbose)
      {
        print ("  Replacing agent_$i.mf algorithm initialization\n");
      }
          
      # replace old algorithm stuff with new
          
      $replacement =~ s/agent.\d+/agent.$i/g;
          
      if ($agent_contents =~ m{agent\.\d+\.algorithm})
      {
        if ($verbose)
        {
          print ("  Agent.algorithm exists. Replacing.\n");
        }
          
        if ($agent_contents =~ s/((\/\/[^\n]*\s+)?agent\..*\s+)+/$replacement/)
        {
          if ($verbose)
          {
            print ("    Algorithm information successfully replaced.\n");
          }
        }
        else
        {
          if ($verbose)
          {
            print ("    Algorithm information was NOT replaced.\n");
          }
        }
      }
      else
      {
        if ($verbose)
        {
          print ("  Agent.algorithm does not exist. Appending.\n");
        }
        
        $agent_contents .= $replacement;
      }
          
      if ($verbose)
      {
        print ("  Writing contents to $sim_path/agent_$i.mf\n");
      }
          
      open agent_file, ">$sim_path/agent_$i.mf" or
        die "ERROR: Couldn't open $sim_path/agent_$i.mf for writing\n";
        print agent_file  $agent_contents; 
      close agent_file;
    }
     
    if (not defined $alg_dir)
    {
      $alg_dir = $algorithm;
    }
     
    if (open groups_file, "$algs_templates/$alg_dir/groups.conf")
    {
      my @groups_list;
      if ($verbose)
      {
        print ("  Reading $algs_templates/$alg_dir/groups.conf...\n");
      }
            
      @groups_list = <groups_file>; 
      close groups_file;
        
      # iterate over the groups list to find group names to create
      for my $line (@groups_list)
      {
        if ($verbose)
        {
          print ("  Parsing $line...\n");
        }
          
        if ($line =~ m{([^\s]+)\s*=\s*\[(\d+)\s*(,|->)\s*([0-9a-zA-Z]+)})
        {
          my ($groupname, $groupfirst, $grouplast) = ($1, $2, $4);
        
          if ($verbose)
          {
            print ("  Initially found $1 with agents $2 to $4...\n");
          }
            
          if (not looks_like_number($grouplast))
          {
            $grouplast = $agents - 1;
          }
          
          if (not $groupname =~ m{^group\.})
          {
            $groupname = "group.$groupname";
          }
          
          if ($verbose)
          {
            print ("  Updating group $groupname with agents $2 to $grouplast...\n");
          }
            
          my $common_contents;
          
          open common_file, "$sim_path/common.mf" or
            die "ERROR: Couldn't open $sim_path/common.mf\n"; 
            $common_contents = join("", <common_file>); 
          close common_file;
            
          if ($verbose)
          {
            print ("  Building group string for $sim_path/common.mf\n");
          }
            
          my $replacement;
            
          $replacement .= "\n// Defining group $groupname\n";
            
          if ($transient)
          {
            if ($verbose)
            {
              print ("  Creating transient group for $groupname...\n");
            }
          
            $replacement .= "$groupname.type = 1;\n";
          }
          else 
          {
            $replacement .= "$groupname.members.size = " . ($grouplast - $groupfirst + 1) . ";\n";
          }
          
          my $index = 0;
          for (my $i = $groupfirst; $i <= $grouplast; ++$i, ++$index)
          {
            if ($transient)
            {
              $replacement .= "$groupname.members.agent.$i = 1;\n"
            }
            else
            {
              $replacement .= "$groupname.members.$index = 'agent.$i';\n";
            }
          }
            
          if ($verbose)
          {
            print ("  Group string built\n");
          }
            
          # check if the group exists
          if ($common_contents =~ m{$groupname})
          {
            if ($verbose)
            {
              print ("  Group already exists. Replacing group definition in common.mf.\n");
            }
            
            $common_contents =~ s/\s*(\/\/ Defining group\s+)?($groupname[^\n]*\s*)+/\n$replacement/;
          }
          else
          {
            if ($verbose)
            {
              print ("  Group does not exists. Adding group definition in common.mf.\n");
            }
            
            $common_contents .= "$replacement\n";
          }
            
          open common_file, ">$sim_path/common.mf" or
            die "ERROR: Couldn't open $sim_path/common.mf for writing\n";
            print common_file  $common_contents; 
          close common_file;
        } # end if line matches the group entry
      } # end for loop over group lines
    } #end if we can read from group file
  } #end if algorithm specified
  
  
  # check for default init for region
  if (($randomize or $equidistant) and !$region)
  {
    # read the contents of the $sim_path/env.mf file
    my $env_contents;
    open env_file, "$sim_path/env.mf" or
      die "ERROR: Couldn't open $sim_path/env.mf\n"; 
      $env_contents = join("", <env_file>); 
    close env_file;
    
    # if we find region size, we should find the region points, so just match
    if ($env_contents =~ m{(region\.[^\.]+)\.size})
    {
      $region = $1;
      if ($verbose)
      {
        print ("Defaults init: region = $1\n");
      }
    }
  }
  
  if ($update_vrep)
  {
    # We need to update vrep's bounding box
    if ($verbose)
    {
      print ("Updating the VREP bounding box in env.mf\n");
    }
  
    # read the contents of the $sim_path/env.mf file
    my $env_contents;
    open env_file, "$sim_path/env.mf" or
      die "ERROR: Couldn't open $sim_path/env.mf\n"; 
      $env_contents = join("", <env_file>); 
    close env_file;
    
    # replace the sw position (min_lat, min_lon)
    if ($env_contents =~ s/vrep_sw_position\s*=\s*\[(\-?\d+\.\d+)\s*,\s*(\-?\d+\.\d+)\]/vrep_sw_position = \[$min_lat, $min_lon\]/)
    {
      if ($verbose)
      {
        print ("  Replaced vrep_sw_position with [$min_lat, $min_lon]\n");
      }
    }
    else
    {
      if ($verbose)
      {
        print ("  Unable to replace vrep_sw_position with [$min_lat, $min_lon]\n");
      }
    }
  
    # replace the sw position (min_lat, min_lon)
    if ($env_contents =~ s/vrep_ne_position\s*=\s*\[(\-?\d+\.\d+)\s*,\s*(\-?\d+\.\d+)\]/vrep_ne_position = \[$max_lat, $max_lon\]/)
    {
      if ($verbose)
      {
        print ("  Replaced vrep_ne_position with [$max_lat, $max_lon]\n");
      }
    }
    else
    {
      if ($verbose)
      {
        print ("  Unable to replace vrep_ne_position with [$max_lat, $max_lon]\n");
      }
    }
    
    # We need to update vrep's bounding box
    if ($verbose)
    {
      print ("  Writing to env.mf\n");
    }
  
    # Update the environment file 
    open (env_file, '>', "$sim_path/env.mf");
      print env_file $env_contents;
    close (env_file);
  }
  
  if ($region)
  {
    # A region has been specified
    if ($verbose)
    {
      print ("Reading $sim_path/env.mf for $region information\n");
    }
  
    # read the contents of the $sim_path/env.mf file
    my $env_contents;
    open env_file, "$sim_path/env.mf" or
      die "ERROR: Couldn't open $sim_path/env.mf\n"; 
      $env_contents = join("", <env_file>); 
    close env_file;
    
    my @vertices = ();
    my $bounding_box_set;
    my $env_changed;
    
    if ($min_lat and $min_lon and $max_lat and $max_lon)
    {
      $bounding_box_set = 1;
      if ($verbose)
      {
        print ("  Region points need to be changed\n");
        print ("    min_lat = $min_lat, min_lon = $min_lon\n");
        print ("    min_lat = $min_lat, min_lon = $min_lon\n");
      }
    }
    
    # if we find region size, we should find the region points, so just match
    if ($env_contents =~ m{$region\.size\s*=\s*(\d+)} and $bounding_box_set)
    {
      if ($verbose)
      {
        print ("  Found $region.size. Attempting to substitute region points\n");
      }
      
      if ($env_contents =~ s/$region\.0\s*=\s*\[(\-?\d+\.\d+)\s*,\s*(\-?\d+\.\d+)\]/$region\.0 = \[$min_lat, $min_lon\]/)
      {
        if ($verbose)
        {
          print ("    Replaced $region.0 with [$min_lat, $min_lon]\n");
        }
      }
      else
      {
        if ($verbose)
        {
          print ("    Unable to replace $region.0 with [$min_lat, $min_lon]\n");
        }
      }
      
      if ($env_contents =~ s/$region\.1\s*=\s*\[(\-?\d+\.\d+)\s*,\s*(\-?\d+\.\d+)\]/$region\.1 = \[$min_lat, $max_lon\]/)
      {
        if ($verbose)
        {
          print ("    Replaced $region.1 with [$min_lat, $max_lon]\n");
        }
      }
      else
      {
        if ($verbose)
        {
          print ("    Unable to replace $region.1 with [$min_lat, $max_lon]\n");
        }
      }
      
      if ($env_contents =~ s/$region\.2\s*=\s*\[(\-?\d+\.\d+)\s*,\s*(\-?\d+\.\d+)\]/$region\.2 = \[$max_lat, $max_lon\]/)
      {
        if ($verbose)
        {
          print ("    Replaced $region.2 with [$max_lat, $max_lon]\n");
        }
      }
      else
      {
        if ($verbose)
        {
          print ("    Unable to replace $region.2 with [$max_lat, $max_lon]\n");
        }
      }
      
      if ($env_contents =~ s/$region\.3\s*=\s*\[(\-?\d+\.\d+)\s*,\s*(\-?\d+\.\d+)\]/$region\.3 = \[$max_lat, $min_lon\]/)
      {
        if ($verbose)
        {
          print ("    Replaced $region.3 with [$max_lat, $min_lon]\n");
        }
      }
      else
      {
        if ($verbose)
        {
          print ("    Unable to replace $region.3 with [$max_lat, $min_lon]\n");
        }
      }
      
      $env_changed = 1;
    }
    # if we don't find region size, then add the region to the env file
    elsif ($bounding_box_set)
    {
      if ($verbose)
      {
        print ("  $region does not exist. Adding $region to env.mf\n");
      }
      
      $env_contents .= "
/**
 * Define $region
**/
$region.object_type = 1;
$region.type = 0;
$region.priority = 0;
$region.size = 4;
$region.0 = [$min_lat, $min_lon];
$region.1 = [$min_lat, $max_lon];
$region.2 = [$max_lat, $max_lon];
$region.3 = [$max_lat, $min_lon];\n";

      $env_changed = 1;
    }
    
    if ($env_changed)
    {
      if ($verbose)
      {
        print ("  Updating env.mf with region changes\n");
      }
      
      # Update the environment file if we added region info
      open (env_file, '>', "$sim_path/env.mf");
        print env_file $env_contents;
      close (env_file);
    }
  
    
    # find region size
    if ($env_contents =~ m{$region\.size\s*=\s*(\d+)})
    {
      my $region_size = $1;  
      
      # $num_agents is specific to the range. It is not $agents.
      my $num_agents = $last - $first + 1;
    
      if ($verbose)
      {
        print ("  Found $region.size ($region_size)\n");
      }
    
      # build region vertices lists in @lats and @lons
      for (my $i = 0; $i < $region_size; ++$i)
      {
        if ($env_contents =~ m{$region\.$i\s*=\s*\[(\-?\d+\.\d+)\s*,\s*(\-?\d+\.\d+)\]})
        {
          my @point = ($1, $2);
      
          if (!$min_lat or $min_lat > $1)
          {
            $min_lat = $1;
          }
          
          if (!$max_lat or $max_lat < $1)
          {
            $max_lat = $1;
          }
          
          if (!$min_lon or $min_lon > $2)
          {
            $min_lon = $2;
          }
      
          if (!$max_lon or $max_lon < $2)
          {
            $max_lon = $2;
          }
          
          if ($verbose)
          {
            print ("  Found $region.$i with [$point[0], $point[1]]\n");
          }
          
          $vertices[$i][0] = $point[0];
          $vertices[$i][1] = $point[1];
        } # end found region
        else
        {
          die "  ERROR: Region $region is malformed. Bad vertices\n";
        } # end did not find region
      } # end iterate over vertices
    
      # Equidistant indicates intent for positions to be spread evenly
      if ($equidistant)
      {
        # Things we know:
        # * buffer is used to distance positions from center of region 
        # * we already have min_lat, max_lat, min_lon, max_lon
        # * we know how many agents there are via $first and $last
        # 
        my $cells = ceil (sqrt ($num_agents));
        
        my $lat_displ = ($max_lat - $min_lat) / $cells;
        my $lon_displ = ($max_lon - $min_lon) / $cells;
        my $row = 0;
        my $col = 0;
        
        if ($verbose)
        {
          print ("Generating equidistant points in $region...\n");
          
          if ($invert and $rotate)
          {
            print ("Formation will be inverted and rotated...\n");
          }
          elsif ($invert)
          {
            print ("Formation will be inverted...\n");
          }
          elsif ($rotate)
          {
            print ("Formation will be rotated...\n");
          }
        }
        
        my ($lat, $lon);
        
        # Create gps locations for each of the agents
        for (my $i = $first; $i <= $last; ++$i)
        {
          if (!$invert)
          {
            # start !inverted
            if ($rotate)
            {
              $lat = $max_lat - $lat_displ * $col;
              $lon = $max_lon - $lon_displ * $row;
            } # end rotated
            else
            {
              $lat = $max_lat - $lat_displ * $row;
              $lon = $min_lon + $lon_displ * $col;
            } # end !rotated
          } # end !inverted
          else
          {
            # start inverted
            if ($rotate)
            {
              $lat = $min_lat + $lat_displ * $col;
              $lon = $max_lon - $lon_displ * $row;
            } # end rotated
            else
            {
              $lat = $min_lat + $lat_displ * $row;
              $lon = $min_lon + $lon_displ * $col;
            } # end !rotated
          } # end inverted
          
          if ($verbose)
          {
            print ("  Generating [$lat, $lon] for agent.$i\n");
          }
          
          # we have a valid point. Change the agent's init file.
          my $agent_contents;
          open agent_file, "$sim_path/agent_$i.mf" or
            die "ERROR: Couldn't open $sim_path/agent_$i.mf for reading\n";
            $agent_contents = join("", <agent_file>); 
          close agent_file;
      
          if ($verbose)
          {
            print ("Replacing agent_$i.mf init pos with [$lat, $lon]\n");
          }
          
          # replace lat and lon with our new points
          $agent_contents =~ s/(\.initial_lat\s*=\s*)\-?\d+\.\d+/$1$lat/;
          $agent_contents =~ s/(\.initial_lon\s*=\s*)\-?\d+\.\d+/$1$lon/;
          
          open agent_file, ">$sim_path/agent_$i.mf" or
            die "ERROR: Couldn't open $sim_path/agent_$i.mf for writing\n";
            print agent_file  $agent_contents; 
          close agent_file;
          
          ++$col;
          
          if ($col >= $cells)
          {
            ++$row;
            $col = 0;
            if ($verbose)
            {
              print ("  Reached end of row, proceeding to row $row\n");
            }
          }
        }
      }
      
      elsif ($randomize)
      {
        if ($verbose)
        {
            print ("Generating randomized coordinates...\n");
        }
        
        # Try to create a point in the polygon region
        for (my $i = $first; $i <= $last; ++$i)
        {
          my $lat = ($min_lat + rand ($max_lat - $min_lat));
          my $lon = ($min_lon + rand ($max_lon - $min_lon));
          
          my @point = ($lat, $lon);
          
          if ($verbose)
          {
            print ("  Generated [$lat, $lon] for agent $i...\n");
          }
        
          # until we have a valid point, keep trying
          while (!inside_polygon (\@point, \@vertices))
          {
            if ($verbose)
            {
              print ("  [$lat, $lon] not in region $region. Regenerating...\n");
            }
        
            $lat = ($min_lat + rand ($max_lat - $min_lat));
            $lon = ($min_lon + rand ($max_lon - $min_lon));
            @point = ($lat, $lon);
          
            if ($verbose)
            {
              print ("  Generated [$lat, $lon] for agent $i...\n");
            }
          } # end for while (!inside_polygon)
          
          # we have a valid point. Change the agent's init file.
          my $agent_contents;
          open agent_file, "$sim_path/agent_$i.mf" or
            die "ERROR: Couldn't open $sim_path/agent_$i.mf for reading\n";
            $agent_contents = join("", <agent_file>); 
          close agent_file;
      
          if ($verbose)
          {
            print ("  Replacing agent_$i.mf init pos with [$lat, $lon]\n");
          }
          
          # replace lat and lon with our new points
          $agent_contents =~ s/(\.initial_lat\s*=\s*)\-?\d+\.\d+/$1$lat/;
          $agent_contents =~ s/(\.initial_lon\s*=\s*)\-?\d+\.\d+/$1$lon/;
          
          open agent_file, ">$sim_path/agent_$i.mf" or
            die "ERROR: Couldn't open $sim_path/agent_$i.mf for writing\n";
            print agent_file  $agent_contents; 
          close agent_file;
        } # end iterating over valid agents
      } # end if randomize
      
    } # end if we found a valid region size
    else
    {
      print ("ERROR: Unable to find $region.size\n");
    }
  } # end if a region was specified
  
  # check if the user wants to change VREP ports
  if ($vrep_start_port)
  {
    if ($verbose)
    {
      print ("Starting port specified. Redoing VREP ports.\n");
    }
  
    # Change port numbers in the agent range
    for (my $i = $first, my $j = $vrep_start_port; $i <= $last; ++$i, ++$j)
    {
      # we have a valid point. Change the agent's init file.
      my $agent_contents;
      open agent_file, "$sim_path/agent_$i.mf" or
        die "ERROR: Couldn't open $sim_path/agent_$i.mf for reading\n";
        $agent_contents = join("", <agent_file>); 
      close agent_file;
  
      if ($verbose)
      {
        print ("  Replacing agent_$i.mf vrep_port with $j\n");
      }
      
      # replace port
      $agent_contents =~ s/(\.vrep_port\s*=\s*)\d+/$1$j/;
      
      if ($verbose)
      {
        print ("  Writing change back to agent_$i.mf\n");
      }
      
      open agent_file, ">$sim_path/agent_$i.mf" or
        die "ERROR: Couldn't open $sim_path/agent_$i.mf for writing\n";
        print agent_file  $agent_contents; 
      close agent_file;
    } #end agent $first -> $last
  } #end $starting port
  
  # check if the user wants to change altitudes
  if ($min_height)
  {
    if ($verbose)
    {
      print ("Min height specified. Redoing VREP ports.\n");
    }
  
    # Change port numbers in the agent range
    for (my $i = $first, my $j = $min_height; $i <= $last; ++$i)
    {
      # read the agent's init file.
      my $agent_contents;
      open agent_file, "$sim_path/agent_$i.mf" or
        die "ERROR: Couldn't open $sim_path/agent_$i.mf for reading\n";
        $agent_contents = join("", <agent_file>); 
      close agent_file;
  
      if ($verbose)
      {
        print ("  Replacing agent_$i.mf initial_alt with $j\n");
      }
      
      # replace port
      $agent_contents =~ s/(\.initial_alt\s*=\s*)\d+/$1$j/;
      
      if ($verbose)
      {
        print ("  Writing change back to agent_$i.mf\n");
      }
      
      open agent_file, ">$sim_path/agent_$i.mf" or
        die "ERROR: Couldn't open $sim_path/agent_$i.mf for writing\n";
        print agent_file  $agent_contents; 
      close agent_file;
      
      if ($unique)
      {
        if ($verbose)
        {
          print ("  Unique modifier specified. Increasing by $height_diff.\n");
        }
      
        $j += $height_diff;
      }
    } #end agent $first -> $last
  } #end $vrep_start_port
  
  # check if the user wants bordered areas
  if (scalar @border > 0)
  {
    if ($verbose)
    {
      print ("Changing bordered regions/areas...\n");
    }
    
    my $run_contents;
    my $found;
    
    # read the run file
    open run_file, "$sim_path/run.pl" or
      die "ERROR: Couldn't open $sim_path/run.pl\n"; 
      $run_contents = join("", <run_file>); 
    close run_file;
    
    # check for broadcast transport already existing
    if ($run_contents =~ m{(border\s*=\s*\(\s*((['"][^'"]+['"]\s*[,]?)*\s*+\)))})
    {
      $found = $1;
    }
    else
    {
      print ("  ERROR: Could not locate border in run file.\n");
    }
    
    if ($found && $verbose)
    {
      print ("  Existing border in run.pl: $found\n");
    }
    
    my $borders = "border = ('" . join ("', '", @border) .  "')";
    
    # replace the old border
    if ($run_contents =~ s/(border\s*=\s*\(\s*((['"][^'"]+['"]\s*[,]?)*\s*+\)))/$borders/)
    {
      if ($verbose)
      {
        print ("  New border: $borders\n");
      }
    }
    else
    {
      print ("  ERROR: Failed to substitute new hosts.\n");
    }
    
    if ($verbose)
    {
      print ("  Writing back to $sim_path/run.pl\n");
    }
    
    # write the updated info to the run file
    open run_file, ">$sim_path/run.pl" or
      die "ERROR: Couldn't open $sim_path/run.pl for writing\n";
      print run_file  $run_contents; 
    close run_file;
  }
  
  # check if the user wants to change transports
  if ($duration)
  {
    if ($verbose)
    {
      print ("Changing duration of sim in $sim_path/run.pl...\n");
    }
    
    my $run_contents;
    open run_file, "$sim_path/run.pl" or
      die "ERROR: Couldn't open $sim_path/run.pl\n"; 
      $run_contents = join("", <run_file>); 
    close run_file;
    
    $run_contents =~ s/(duration\s*=\s*)\d+/$1$duration/;  
    
    open run_file, ">$sim_path/run.pl" or
      die "ERROR: Couldn't open $sim_path/run.pl for writing\n";
      print run_file  $run_contents; 
    close run_file;
    
  }
  
  # check if the user wants to change the domain
  if ($domain)
  {
    if ($verbose)
    {
      print ("Changing domain in $sim_path/run.pl...\n");
    }
    
    my $run_contents;
    open run_file, "$sim_path/run.pl" or
      die "ERROR: Couldn't open $sim_path/run.pl\n"; 
      $run_contents = join("", <run_file>); 
    close run_file;
    
    if ($verbose)
    {
      print ("  Changing network domain to $domain\n");
    }
    $run_contents =~ s/(domain\s*=\s*)['"][^'"]+['"]/$1\"$domain\"/;  
  
    open run_file, ">$sim_path/run.pl" or
      die "ERROR: Couldn't open $sim_path/run.pl for writing\n";
      print run_file  $run_contents; 
    close run_file;
  }
  
  # check if the user wants to change debug levels
  if ($madara_debug or $gams_debug)
  {
    if ($verbose)
    {
      print ("Changing debug levels in $sim_path/run.pl...\n");
    }
    
    my $run_contents;
    open run_file, "$sim_path/run.pl" or
      die "ERROR: Couldn't open $sim_path/run.pl\n"; 
      $run_contents = join("", <run_file>); 
    close run_file;
    
    if ($madara_debug)
    {
      if ($verbose)
      {
        print ("  Changing MADARA debug level to $madara_debug\n");
      }
      $run_contents =~ s/(madara_debug\s*=\s*)\d+/$1$madara_debug/;  
    }
    
    if ($gams_debug)
    {
      if ($verbose)
      {
        print ("  Changing GAMS debug level to $gams_debug\n");
      }
      $run_contents =~ s/(gams_debug\s*=\s*)\d+/$1$gams_debug/; 
    }
    
    open run_file, ">$sim_path/run.pl" or
      die "ERROR: Couldn't open $sim_path/run.pl for writing\n";
      print run_file  $run_contents; 
    close run_file;
    
  }
  
  # check if the user wants to change transports
  if ($multicast or $broadcast or scalar @udp > 0)
  {
    if ($verbose)
    {
      print ("Changing transport...\n");
    }
    
    my $found;
    my $hosts;
    my $transport;
    my $run_contents;
    
    open run_file, "$sim_path/run.pl" or
      die "ERROR: Couldn't open $sim_path/run.pl\n"; 
      $run_contents = join("", <run_file>); 
    close run_file;
    
    # check for broadcast transport already existing
    if ($run_contents =~ m{(hosts\s*=\s*\(\s*(['"][^'"]+['"]\s*[,]?\s*)+\))})
    {
      $found = $1;
    }
    else
    {
      print ("  ERROR: Could not locate hosts.\n");
    }
    
    if ($found && $verbose)
    {
      print ("  Existing hosts in run.pl: $found\n");
    }
      
    if ($broadcast)
    {
      $hosts = "hosts = ('$broadcast')";
      $transport = "broadcast";
    }
    elsif (scalar @udp > 0)
    {
      $hosts = "hosts = ('" . join ("', '", @udp) .  "')";
      $transport = "udp";
    }
    else
    {
      $hosts = "hosts = ('$multicast')";
      $transport = "multicast";
    }
  
    if ($verbose)
    {
      print ("  Attempting to substitute $hosts\n");
    }
      
    if ($run_contents =~ s/(hosts\s*=\s*\(\s*(['"][^'"]+['"]\s*[,]?\s*)+\))/$hosts/)
    {
      if ($verbose)
      {
        print ("  New hosts: $hosts\n");
      }
    }
    else
    {
      print ("  ERROR: Failed to substitute new hosts.\n");
    }
      
    if ($verbose)
    {
      print ("  Attempting to substitute $transport\n");
    }
    
    if ($run_contents =~ s/multicast/$transport/)
    {
      if ($verbose)
      {
        print ("  Transport change: multicast -> $transport\n");
      }
    }
    elsif ($run_contents =~ s/udp/$transport/)
    {
      if ($verbose)
      {
        print ("  Transport change: udp -> $transport\n");
      }
    }
    elsif ($run_contents =~ s/broadcast/$transport/)
    {
      if ($verbose)
      {
        print ("  Transport change: broadcast -> $transport\n");
      }
    }
    else
    {
      print (" ERROR: Unable to replace transport option.");
    }
      
    if ($verbose)
    {
      print ("  Writing back to $sim_path/run.pl\n");
    }
    
    open run_file, ">$sim_path/run.pl" or
      die "ERROR: Couldn't open $sim_path/run.pl for writing\n";
      print run_file  $run_contents; 
    close run_file;
  }
  
  # if the user has asked for a group to be created/modified/used
  if ($group)
  { 
    if ($verbose)
    {
      print ("Updating group $group...\n");
    }
    
    my $common_contents;
  
    open common_file, "$sim_path/common.mf" or
      die "ERROR: Couldn't open $sim_path/common.mf\n"; 
      $common_contents = join("", <common_file>); 
    close common_file;
    
    if ($verbose)
    {
      print ("  Building group string for $sim_path/common.mf\n");
    }
    
    my $replacement;
    
    $replacement .= "\n// Defining group $group\n";
    
    if ($transient)
    {
      if ($verbose)
      {
        print ("  Creating transient group for $group...\n");
      }
          
      $replacement .= "$group.type = 1;\n";
    }
    else 
    {
      $replacement .= "$group.members.size = " . ($last - $first + 1) . ";\n";
    }
          
    my $index = 0;
    for (my $i = $first; $i <= $last; ++$i, ++$index)
    {
      if ($transient)
      {
        $replacement .= "$group.members.agent.$i = 1;\n"
      }
      else
      {
        $replacement .= "$group.members.$index = 'agent.$i';\n";
      }
    }
    
    if ($verbose)
    {
      print ("  Group string built\n");
    }
    
    # check if the group exists
    if ($common_contents =~ m{$group})
    {
      if ($verbose)
      {
        print ("  Group already exists. Replacing group definition in common.mf.\n");
      }
    
      $common_contents =~ s/(\/\/ Defining group\s+)?($group[^\n]*\s*)+/$replacement/;
    }
    else
    {
      if ($verbose)
      {
        print ("  Group does not exists. Adding group definition in common.mf.\n");
      }
    
      $common_contents .= "$replacement\n";
    }
    
    open common_file, ">$sim_path/common.mf" or
      die "ERROR: Couldn't open $sim_path/common.mf for writing\n";
      print common_file  $common_contents; 
    close common_file;
  }
  
  # make new container
  foreach my $new_container (@containers)
  {
    if ($new_container and not -f "$containers_path/$new_container.h")
    {
      my $new_container_uc = uc $new_container;
    
      if ($verbose)
      {
        print ("Adding infrastructure for container $new_container...\n");
      }
         
      # Create file contents for custom algorithms
       
      my $container_header_contents = "
#ifndef   _CONTAINERS_${new_container_uc}_H_
#define   _CONTAINERS_${new_container_uc}_H_

#include <string>
#include <vector>

#include \"madara/knowledge/KnowledgeBase.h\"
#include \"madara/knowledge/containers/Integer.h\"
#include \"madara/knowledge/containers/Double.h\"
#include \"madara/knowledge/containers/NativeIntegerVector.h\"
#include \"madara/knowledge/containers/NativeDoubleVector.h\"
#include \"madara/knowledge/containers/String.h\"

namespace containers
{
  /**
  * Houses variables modified and read by threads
  **/
  class ${new_container}
  {
  public:
    /**
     * Default constructor
     **/
    ${new_container} ();

    /**
     * Constructor
     * \@param  knowledge    the context containing variables and values
     **/
    ${new_container} (
      madara::knowledge::KnowledgeBase & knowledge);

    /**
     * Initializes all containers and variable values
     * \@param  knowledge    the context containing variables and values
     **/
    void init (madara::knowledge::KnowledgeBase & knowledge);

    /**
     * Modifies all containers to indicate they should be resent on next send
     **/
    void modify (void);

    /**
     * Reads all MADARA containers into the user-facing C++ variables.
     **/
    void read (void);

    /**
     * Writes user-facing C++ variables to the MADARA containers. Note
     * that this does not check if values are different. It always overwrites
     * the values in the knowledge base with what is currently in local vars
     **/
    void write (void);

    /// imu acceleration x, y, z
    std::vector<double> imu_sigma_accel;

    /// orientation x, y, z (roll, pitch, yaw)
    std::vector<double> orientation;

    /// the position of the agent (x, y, z)
    std::vector<double> position;

  private:

    /// imu_sigma_accel that directly interfaces to the KnowledgeBase
    madara::knowledge::containers::NativeDoubleArray imu_sigma_accel_;

    /// orientation that directly interfaces to the KnowledgeBase
    madara::knowledge::containers::NativeDoubleArray orientation_;

    /// imu_sigma_accel that directly interfaces to the KnowledgeBase
    madara::knowledge::containers::NativeDoubleArray position_;


    /// unmanaged context for locking. The knowledge base should stay in scope
    madara::knowledge::ThreadSafeContext * context_;
  };

} // end containers namespace

#endif // _CONTAINERS_${new_container_uc}_H_
";

      my $container_source_contents = "

#include \"${new_container}.h\"

#include \"madara/knowledge/ContextGuard.h\"

containers::${new_container}::${new_container} ()
: context_ (0)
{
}

containers::${new_container}::${new_container} (
  madara::knowledge::KnowledgeBase & knowledge)
{
  init (knowledge);
}

void
containers::${new_container}::init (
  madara::knowledge::KnowledgeBase & knowledge)
{
  // hold context for use in guards later
  context_ = &knowledge.get_context ();

  // init knowledge variables. DO NOT REMOVE COMMENT
  imu_sigma_accel_.set_name (\".imu.sigma.accel\", knowledge);
  orientation_.set_name (\".orientation\", knowledge);
  position_.set_name (\".position\", knowledge);
}

void
containers::${new_container}::read (void)
{
  if (context_)
  {
    // lock the context for consistency
    madara::knowledge::ContextGuard guard (*context_);

    // update user-facing variables. DO NOT REMOVE COMMENT
    imu_sigma_accel = imu_sigma_accel_.to_record ().to_doubles ();
    orientation = orientation_.to_record ().to_doubles ();
    position = position_.to_record ().to_doubles ();
  }
}

void
containers::${new_container}::write (void)
{
  if (context_)
  {
    // lock the context for consistency
    madara::knowledge::ContextGuard guard (*context_);

    // update knowledge base. DO NOT REMOVE COMMENT
    imu_sigma_accel_.set (imu_sigma_accel);
    orientation_.set (orientation);
    position_.set (position);
  }
}

void
containers::${new_container}::modify (void)
{
  // mark containers as modified so the values are resent. DO NOT REMOVE COMMENT
  imu_sigma_accel_.modify ();
  orientation_.modify ();
  position_.modify ();
}
";

   
      # open files for writing
      open container_header, ">$containers_path/$new_container.h" or 
        die "ERROR: Couldn't open $containers_path/$new_container.h for writing\n";
        print container_header $container_header_contents;
      close container_header;
          
      open container_source, ">$containers_path/$new_container.cpp" or 
        die "ERROR: Couldn't open $containers_path/$new_container.cpp for writing\n";
        print container_source $container_source_contents;
      close container_source;
          
    }
  }

  if (scalar @new_algorithm > 0 or scalar @new_platform > 0
      or scalar @new_thread > 0 or scalar @new_transport > 0
      or scalar @new_platform_thread > 0
      or scalar @new_receive_filters > 0
      or scalar @new_send_filters > 0)
  {
    # filters directory was not in 1.0, so try to be backwards compatible
    if (not -d "$filters_path")
    {
      make_path("$filters_path");
      
      my $project_contents;
      
      open project_file, "$path/project.mpc" or 
        die "ERROR: Couldn't open $path/project.mpc for reading\n";
        $project_contents = join("", <project_file>); 
      close project_file;
      
      if (not $project_contents =~ /src\/filters/)
      {
        $project_contents =~ s/src\/algorithms/src\/algorithms\n    src\/filters/g;
      }
      
      open project_file, ">$path/project.mpc" or 
        die "ERROR: Couldn't open $path/project.mpc for writing\n";
        print project_file $project_contents; 
      close project_file;
    }
  
    if (not -f "$path/using_boost.mpb")
    {
      my $gamsroot = $ENV{GAMS_ROOT};
      
      if ($verbose)
      {
        print ("  Copying base projects from $gamsroot to $path...\n");
      }
      
      copy "$gamsroot/using_vrep.mpb", "$path/";
      #copy "$gamsroot/using_madara.mpb", "$path/";
      #copy "$gamsroot/using_gams.mpb", "$path/";
      #copy "$gamsroot/using_boost.mpb", "$path/";
    }
  
    foreach my $new_alg (@new_algorithm)
    {
      if ($new_alg and not -f "$algs_path/$new_alg.h")
      {
        my $new_alg_uc = uc $new_alg;
      
        if ($verbose)
        {
          print ("Adding infrastructure for algorithm $new_alg...\n");
        }
         
        # Create file contents for custom algorithms
       
        my $alg_header_contents = "
#ifndef   _ALGORITHM_${new_alg_uc}_H_
#define   _ALGORITHM_${new_alg_uc}_H_

#include <string>

#include \"madara/knowledge/containers/Integer.h\"

#include \"gams/variables/Sensor.h\"
#include \"gams/platforms/BasePlatform.h\"
#include \"gams/variables/AlgorithmStatus.h\"
#include \"gams/variables/Self.h\"
#include \"gams/algorithms/BaseAlgorithm.h\"
#include \"gams/algorithms/AlgorithmFactory.h\"

namespace algorithms
{
  /**
  * A custom algorithm generated by gpc.pl
  **/
  class $new_alg : public gams::algorithms::BaseAlgorithm
  {
  public:
    /**
     * Constructor
     * \@param  knowledge    the context containing variables and values
     * \@param  platform     the underlying platform the algorithm will use
     * \@param  sensors      map of sensor names to sensor information
     * \@param  self         self-referencing variables
     **/
    $new_alg (
      madara::knowledge::KnowledgeBase * knowledge = 0,
      gams::platforms::BasePlatform * platform = 0,
      gams::variables::Sensors * sensors = 0,
      gams::variables::Self * self = 0,
      gams::variables::Agents * agents = 0);

    /**
     * Destructor
     **/
    virtual ~$new_alg ();

    /**
     * Analyzes environment, platform, or other information
     * \@return bitmask status of the platform. \@see Status.
     **/
    virtual int analyze (void);
      
    /**
     * Plans the next execution of the algorithm
     * \@return bitmask status of the platform. \@see Status.
     **/
    virtual int execute (void);

    /**
     * Plans the next execution of the algorithm
     * \@return bitmask status of the platform. \@see Status.
     **/
    virtual int plan (void);
  };

  /**
   * A factory class for creating ${new_alg} Algorithms
   **/
  class ${new_alg}Factory : public gams::algorithms::AlgorithmFactory
  {
  public:
     /**
     * Creates a ${new_alg} algorithm.
     * \@param   args      the args passed with the 
     * \@param   knowledge the knowledge base to use
     * \@param   platform  the platform. This will be set by the
     *                     controller in init_vars.
     * \@param   sensors   the sensor info. This will be set by the
     *                     controller in init_vars.
     * \@param   self      self-referencing variables. This will be
     *                     set by the controller in init_vars
     * \@param   agents    the list of agents, which is dictated by
     *                     init_vars when a number of processes is set. This
     *                     will be set by the controller in init_vars
     **/
    virtual gams::algorithms::BaseAlgorithm * create (
      const madara::knowledge::KnowledgeMap & args,
      madara::knowledge::KnowledgeBase * knowledge,
      gams::platforms::BasePlatform * platform,
      gams::variables::Sensors * sensors,
      gams::variables::Self * self,
      gams::variables::Agents * agents);
  };
} // end algorithms namespace

#endif // _ALGORITHM_${new_alg_uc}_H_
";
      
        my $alg_source_contents = " 
#include \"${new_alg}.h\"

#include <iostream>

gams::algorithms::BaseAlgorithm *
algorithms::${new_alg}Factory::create (
  const madara::knowledge::KnowledgeMap & /*args*/,
  madara::knowledge::KnowledgeBase * knowledge,
  gams::platforms::BasePlatform * platform,
  gams::variables::Sensors * sensors,
  gams::variables::Self * self,
  gams::variables::Agents * agents)
{
  gams::algorithms::BaseAlgorithm * result (0);
  
  if (knowledge && sensors && platform && self)
  {
    result = new ${new_alg} (knowledge, platform, sensors, self);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      \"algorithms::${new_alg}Factory::create:\" \
      \" failed to create due to invalid pointers. \" \
      \" knowledge=%p, sensors=%p, platform=%p, self=%p, agents=%p\\n\",
      knowledge, sensors, platform, self, agents);
  }

  /**
   * Note the usage of logger macros with the GAMS global logger. This
   * is highly optimized and is just an integer check if the log level is
   * not high enough to print the message
   **/
  if (result == 0)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      \"algorithms::${new_alg}Factory::create:\" \
      \" unknown error creating ${new_alg} algorithm\\n\");
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      \"algorithms::${new_alg}Factory::create:\" \
      \" successfully created ${new_alg} algorithm\\n\");
  }

  return result;
}

algorithms::${new_alg}::${new_alg} (
  madara::knowledge::KnowledgeBase * knowledge,
  gams::platforms::BasePlatform * platform,
  gams::variables::Sensors * sensors,
  gams::variables::Self * self,
  gams::variables::Agents * agents)
  : gams::algorithms::BaseAlgorithm (knowledge, platform, sensors, self, agents)
{
  status_.init_vars (*knowledge, \"${new_alg}\", self->agent.prefix);
  status_.init_variable_values ();
}

algorithms::${new_alg}::~${new_alg} ()
{
}

int
algorithms::${new_alg}::analyze (void)
{
  return 0;
}
      

int
algorithms::${new_alg}::execute (void)
{
  return 0;
}


int
algorithms::${new_alg}::plan (void)
{
  return 0;
}
";
           
        # open files for writing
        open alg_header, ">$algs_path/$new_alg.h" or 
          die "ERROR: Couldn't open $algs_path/$new_alg.h for writing\n";
          print alg_header $alg_header_contents;
        close alg_header;
          
        open alg_source, ">$algs_path/$new_alg.cpp" or 
          die "ERROR: Couldn't open $algs_path/$new_alg.cpp for writing\n";
          print alg_source $alg_source_contents;
        close alg_source;
          
         
        # open the individual agent files and update them with the algorithm

        if ($verbose)
        {
          print ("  Replacing agent[$first - $last] algorithms with $new_alg\n");
        }
            
        for (my $i = $first; $i <= $last; ++$i)
        {
          if ($verbose)
          {
            print ("    Replacing agent[$i] algorithm with $new_alg\n");
          }
          
          my $agent_contents;
          open agent_file, "$sim_path/agent_$i.mf" or
            die "ERROR: Couldn't open $sim_path/agent_$i.mf for reading\n";
            $agent_contents = join("", <agent_file>); 
          close agent_file;
        
          # replace lat and lon with our new points
          $agent_contents =~ s/(\.algorithm\s*=\s*)\-?[^;]+/$1\"$new_alg\"/;
            
          if ($verbose)
          {
            print ("    Writing agent_$i.mf\n");
          }
          
          open agent_file, ">$sim_path/agent_$i.mf" or
            die "ERROR: Couldn't open $sim_path/agent_$i.mf for writing\n";
            print agent_file  $agent_contents; 
          close agent_file;
        } # end for range from first to last    
      } # end if new algorithm and the source files don't exist
    } #end foreach new algorithm
    
    foreach my $new_thr (@new_platform_thread)
    {
      if ($new_thr and not -f "$plats_threads_path/$new_thr.h")
      {
        my $new_thr_uc = uc $new_thr;
      
        if ($verbose)
        {
          print ("Adding infrastructure for thread $new_thr...\n");
        }
         
        # Create file contents for custom thread
       
        my $header_contents = "
#ifndef   _PLATFORM_THREAD_${new_thr_uc}_H_
#define   _PLATFORM_THREAD_${new_thr_uc}_H_

#include <string>

#include \"madara/threads/BaseThread.h\"

namespace platforms
{
  namespace threads
  {
    /**
    * A custom thread generated by gpc.pl
    **/
    class ${new_thr} : public madara::threads::BaseThread
    {
    public:
      /**
       * Default constructor
       **/
      ${new_thr} ();
      
      /**
       * Destructor
       **/
      virtual ~${new_thr} ();
      
      /**
        * Initializes thread with MADARA context
        * \@param   knowledge   context for querying current program state
        **/
      virtual void init (madara::knowledge::KnowledgeBase & knowledge);

      /**
        * Executes the main thread logic
        **/
      virtual void run (void);

    private:
      /// data plane if we want to access the knowledge base
      madara::knowledge::KnowledgeBase data_;
    };
  } // end namespace threads
} // end namespace platforms

#endif // _PLATFORM_THREAD_${new_thr_uc}_H_
";
        my $source_contents = "
#include \"gams/loggers/GlobalLogger.h\"
#include \"${new_thr}.h\"

namespace knowledge = madara::knowledge;

// constructor
platforms::threads::${new_thr}::${new_thr} ()
{
}

// destructor
platforms::threads::${new_thr}::~${new_thr} ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
platforms::threads::${new_thr}::init (knowledge::KnowledgeBase & knowledge)
{
  // point our data plane to the knowledge base initializing the thread
  data_ = knowledge;
}

/**
 * Executes the actual thread logic. Best practice is to simply do one loop
 * iteration. If you want a long running thread that executes something
 * frequently, see the madara::threads::Threader::runHz method in your
 * controller.
 **/
void
platforms::threads::${new_thr}::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    \"platforms::threads::${new_thr}::run:\" \
    \" executing\\n\");
}
";
    
        # open files for writing
        open thread_header, ">$plats_threads_path/$new_thr.h" or 
          die "ERROR: Couldn't open $plats_threads_path/$new_thr.h for writing\n";
          print thread_header $header_contents;
        close thread_header;
          
        open thread_source, ">$plats_threads_path/$new_thr.cpp" or 
          die "ERROR: Couldn't open $plats_threads_path/$new_thr.cpp for writing\n";
          print thread_source $source_contents;
        close thread_source;
      }
    }

    # get a list of all custom platform threads
    @platform_threads = glob "$plats_threads_path/*.cpp";
    for (my $i = 0; $i < scalar @platform_threads; ++$i)
    {
      my ($file, $dir, $suffix) = fileparse($platform_threads[$i], qr/\.[^.]*/);
      $platform_threads[$i] = $file;
    }
    
    foreach my $new_plat (@new_platform)
    {
      if ($new_plat and not -f "$plats_path/$new_plat.h")
      {
        my $new_plat_uc = uc $new_plat;
        if ($verbose)
        {
          print ("Adding infrastructure for thread $new_plat...\n");
        }
         
        # Create file contents for custom platform
       
        my $header_contents = "
#ifndef   _PLATFORM_${new_plat_uc}_H_
#define   _PLATFORM_${new_plat_uc}_H_

#include \"gams/platforms/BasePlatform.h\"
#include \"gams/platforms/PlatformFactory.h\"
#include \"madara/threads/Threader.h\"
#include \"gams/pose/GPSFrame.h\"
#include \"gams/pose/CartesianFrame.h\"

namespace platforms
{        
  /**
  * A custom platform generated by gpc.pl
  **/
  class ${new_plat} : public gams::platforms::BasePlatform
  {
  public:
    /**
     * Constructor
     * \@param  knowledge  context containing variables and values
     * \@param  sensors    map of sensor names to sensor information
     * \@param  self       self referencing variables for the agent
     **/
    ${new_plat} (
      madara::knowledge::KnowledgeBase * knowledge = 0,
      gams::variables::Sensors * sensors = 0,
      gams::variables::Self * self = 0);

    /**
     * Destructor
     **/
    virtual ~${new_plat} ();

    /**
     * Polls the sensor environment for useful information. Required.
     * \@return number of sensors updated/used
     **/
    virtual int sense (void);

    /**
     * Analyzes platform information. Required.
     * \@return bitmask status of the platform. \@see PlatformAnalyzeStatus.
     **/
    virtual int analyze (void);

    /**
     * Gets the name of the platform. Required.
     **/
    virtual std::string get_name () const;

    /**
     * Gets the unique identifier of the platform. This should be an
     * alphanumeric identifier that can be used as part of a MADARA
     * variable (e.g. vrep_ant, autonomous_snake, etc.) Required.
     * \@return the id of the platform to use in factory methods
     **/
    virtual std::string get_id () const;

    /**
     * Gets the position accuracy in meters. Optional.
     * \@return position accuracy
     **/
    virtual double get_accuracy (void) const;

    /**
     * Gets Location of platform, within its parent frame. Optional.
     * \@return Location of platform
     **/
    gams::pose::Position get_location (void) const;

    /**
     * Gets Rotation of platform, within its parent frame. Optional.
     * \@return Location of platform
     **/
    gams::pose::Orientation get_orientation (void) const;

    /**
     * Gets sensor radius. Optional.
     * \@return minimum radius of all available sensors for this platform
     **/
    virtual double get_min_sensor_range (void) const;

    /**
     * Gets move speed. Optional.
     * \@return speed in meters per second
     **/
    virtual double get_move_speed (void) const;

    /**
     * Instructs the agent to return home. Optional.
     * \@return the status of the home operation, \@see PlatformReturnValues
     **/
    virtual int home (void);

    /**
     * Instructs the agent to land. Optional.
     * \@return the status of the land operation, \@see PlatformReturnValues
     **/
    virtual int land (void);

    /**
     * Moves the platform to a location. Optional.
     * \@param   location    the coordinates to move to
     * \@param   epsilon     approximation value
     * \@return the status of the move operation, \@see PlatformReturnValues
     **/
    virtual int move (const gams::pose::Position & location,
      double epsilon = 0.1);

    /**
     * Rotates the platform to match a given angle. Optional.
     * \@param   target    the rotation to move to
     * \@param   epsilon   approximation value
     * \@return the status of the rotate, \@see PlatformReturnValues
     **/
    virtual int rotate (const gams::pose::Orientation & target,
      double epsilon = M_PI/16);

    /**
     * Moves the platform to a pose (location and rotation). Optional.
     *
     * This default implementation calls move and rotate with the
     * Location and Rotation portions of the target Pose. The return value
     * is composed as follows: if either call returns ERROR (0), this call
     * also returns ERROR (0). Otherwise, if BOTH calls return ARRIVED (2),
     * this call also returns ARRIVED (2). Otherwise, this call returns
     * MOVING (1)
     *
     * Overrides might function differently.
     *
     * \@param   target        the coordinates to move to
     * \@param   loc_epsilon   approximation value for the location
     * \@param   rot_epsilon   approximation value for the rotation
     * \@return the status of the operation, \@see PlatformReturnValues
     **/
    virtual int pose (const gams::pose::Pose & target,
      double loc_epsilon = 0.1, double rot_epsilon = M_PI/16);

    /**
     * Pauses movement, keeps source and dest at current values. Optional.
     **/
    virtual void pause_move (void);

    /**
     * Set move speed. Optional.
     * \@param speed new speed in meters/second
     **/
    virtual void set_move_speed (const double& speed);

    /**
     * Stops movement, resetting source and dest to current location.
     * Optional.
     **/
    virtual void stop_move (void);

    /**
     * Instructs the agent to take off. Optional.
     * \@return the status of the takeoff, \@see PlatformReturnValues
     **/
    virtual int takeoff (void);
    
    /**
     * Returns the world reference frame for the platform (e.g. GPS or cartesian)
     **/
    virtual const gams::pose::ReferenceFrame & get_frame (void) const;
    
  private:
    // a threader for managing platform threads
    madara::threads::Threader threader_;    
  }; // end ${new_plat} class
    

  /**
   * A factory class for creating ${new_plat} platforms
   **/
  class ${new_plat}Factory : public gams::platforms::PlatformFactory
  {
  public:
    /**
     * Creates a ${new_plat} platform.
     * \@param   args      no arguments are necessary for this platform
     * \@param   knowledge the knowledge base. This will be set by the
     *                    controller in init_vars.
     * \@param   sensors   the sensor info. This will be set by the
     *                    controller in init_vars.
     * \@param   platforms status inform for all known agents. This
     *                    will be set by the controller in init_vars
     * \@param   self      self-referencing variables. This will be
     *                    set by the controller in init_vars
     **/
    virtual gams::platforms::BasePlatform * create (
      const madara::knowledge::KnowledgeMap & args,
      madara::knowledge::KnowledgeBase * knowledge,
      gams::variables::Sensors * sensors,
      gams::variables::Platforms * platforms,
      gams::variables::Self * self);
  };
    
} // end platforms namespace

#endif // _PLATFORM_${new_plat_uc}_H_
"; 
        
        my $source_contents = "
#include \"madara/knowledge/containers/NativeDoubleVector.h\"
#include \"${new_plat}.h\"\n";


        for my $new_thr (@platform_threads)
        {
          $source_contents .= "#include \"threads/$new_thr.h\"\n";
        }

        $source_contents .= "
 
// factory class for creating a ${new_plat} 
gams::platforms::BasePlatform *
platforms::${new_plat}Factory::create (
        const madara::knowledge::KnowledgeMap & args,
        madara::knowledge::KnowledgeBase * knowledge,
        gams::variables::Sensors * sensors,
        gams::variables::Platforms * platforms,
        gams::variables::Self * self)
{
  return new ${new_plat} (knowledge, sensors, self);
}
        
// Constructor
platforms::${new_plat}::${new_plat} (
  madara::knowledge::KnowledgeBase * knowledge,
  gams::variables::Sensors * sensors,
  gams::variables::Self * self)
: gams::platforms::BasePlatform (knowledge, sensors, self)
{
  // as an example of what to do here, create a coverage sensor
  if (knowledge && sensors)
  {
    // set the data plane for the threader
    threader_.set_data_plane (*knowledge);
  
    // create a coverage sensor
    gams::variables::Sensors::iterator it = sensors->find (\"coverage\");
    if (it == sensors->end ()) // create coverage sensor
    {
      // get origin
      gams::pose::Position origin (gams::pose::gps_frame());
      madara::knowledge::containers::NativeDoubleArray origin_container;
      origin_container.set_name (\"sensor.coverage.origin\", *knowledge, 3);
      origin.from_container (origin_container);

      // establish sensor
      gams::variables::Sensor* coverage_sensor =
        new gams::variables::Sensor (\"coverage\", knowledge, 2.5, origin);
      (*sensors)[\"coverage\"] = coverage_sensor;
    }
    (*sensors_)[\"coverage\"] = (*sensors)[\"coverage\"];
    status_.init_vars (*knowledge, get_id ());
    
    // create threads";
    
        for my $new_thr (@platform_threads)
        {
          $source_contents .= "
    threader_.run(1.0, \"${new_thr}\", new threads::${new_thr}());";
        }
        $source_contents .= "
    // end create threads
    
    
    /**
    * the following should be set when movement is available in your
    * platform. If on construction, movement should be possible, then
    * feel free to keep this uncommented. Otherwise, set it somewhere else
    * in analyze or somewhere else when appropriate to enable movement.
    * If you never enable movement_available, movement based algorithms are
    * unlikely to ever move with your platform.
    **/
    status_.movement_available = 1;
  }
}


// Destructor
platforms::${new_plat}::~${new_plat} ()
{
  threader_.terminate ();
  threader_.wait ();
}


// Polls the sensor environment for useful information. Required.
int platforms::${new_plat}::sense (void)
{
  return gams::platforms::PLATFORM_OK;
}


// Analyzes platform information. Required.
int
platforms::${new_plat}::analyze (void)
{
  return gams::platforms::PLATFORM_OK;
}


// Gets the name of the platform. Required.
std::string
platforms::${new_plat}::get_name (void) const
{
  return \"${new_plat}\";
}


// Gets the unique identifier of the platform.
std::string
platforms::${new_plat}::get_id (void) const
{
  return \"${new_plat}\";
}


// Gets the position accuracy in meters. Optional.
double
platforms::${new_plat}::get_accuracy (void) const
{
  // will depend on your localization capabilities for robotics
  return 0.0;
}

// Gets Location of platform, within its parent frame. Optional.
gams::pose::Position
platforms::${new_plat}::get_location (void) const
{
  gams::pose::Position result;
  
  return result;
}


// Gets Rotation of platform, within its parent frame. Optional.
gams::pose::Orientation
platforms::${new_plat}::get_orientation (void) const
{
  gams::pose::Orientation result;
  
  return result;
}


// Gets sensor radius. Optional.
double
platforms::${new_plat}::get_min_sensor_range (void) const
{
  // should be in square meters
  return 0.0;
}

// Gets move speed. Optional.
double
platforms::${new_plat}::get_move_speed (void) const
{
  // should be in meters/s
  return 0.0;
}

// Instructs the agent to return home. Optional.
int
platforms::${new_plat}::home (void)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  return gams::platforms::PLATFORM_IN_PROGRESS;
}


// Instructs the agent to land. Optional.
int
platforms::${new_plat}::land (void)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  return gams::platforms::PLATFORM_IN_PROGRESS;
}


// Moves the platform to a location. Optional.
int
platforms::${new_plat}::move (
  const gams::pose::Position & location,
  double epsilon)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  return gams::platforms::PLATFORM_MOVING;
}


// Rotates the platform to match a given angle. Optional.
int
platforms::${new_plat}::rotate (
  const gams::pose::Orientation & target,
  double epsilon)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  return gams::platforms::PLATFORM_MOVING;
}


// Moves the platform to a pose (location and rotation). Optional.
int
platforms::${new_plat}::pose (const gams::pose::Pose & target,
  double loc_epsilon, double rot_epsilon)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  return gams::platforms::PLATFORM_MOVING;
}

// Pauses movement, keeps source and dest at current values. Optional.
void
platforms::${new_plat}::pause_move (void)
{
}


// Set move speed. Optional.
void
platforms::${new_plat}::set_move_speed (const double& speed)
{
}


// Stops movement, resetting source and dest to current location. Optional.
void
platforms::${new_plat}::stop_move (void)
{
}

// Instructs the agent to take off. Optional.
int
platforms::${new_plat}::takeoff (void)
{
  return gams::platforms::PLATFORM_OK;
}

const gams::pose::ReferenceFrame &
platforms::${new_plat}::get_frame (void) const
{
  // For cartesian, replace with gams::pose::default_frame()
  return gams::pose::gps_frame();
}
";
      
        # open files for writing
        open platform_header, ">$plats_path/$new_plat.h" or 
          die "ERROR: Couldn't open $plats_path/$new_plat.h for writing\n";
          print platform_header $header_contents;
        close platform_header;
          
        open platform_source, ">$plats_path/$new_plat.cpp" or 
          die "ERROR: Couldn't open $plats_path/$new_plat.cpp for writing\n";
          print platform_source $source_contents;
        close platform_source;
      } # end if platform does not exist yet
    } # end foreach new platform
    
    foreach my $new_thr (@new_thread)
    {
      if ($new_thr and not -f "$threads_path/$new_thr.h")
      {
        my $new_thr_uc = uc $new_thr;
      
        if ($verbose)
        {
          print ("Adding infrastructure for thread $new_thr...\n");
        }
         
        # Create file contents for custom thread
       
        my $header_contents = "
#ifndef   _THREAD_${new_thr_uc}_H_
#define   _THREAD_${new_thr_uc}_H_

#include <string>

#include \"madara/threads/BaseThread.h\"

namespace threads
{
  /**
  * A custom thread generated by gpc.pl
  **/
  class ${new_thr} : public madara::threads::BaseThread
  {
  public:
    /**
     * Default constructor
     **/
    ${new_thr} ();
    
    /**
     * Destructor
     **/
    virtual ~${new_thr} ();
    
    /**
      * Initializes thread with MADARA context
      * \@param   context   context for querying current program state
      **/
    virtual void init (madara::knowledge::KnowledgeBase & knowledge);

    /**
      * Executes the main thread logic
      **/
    virtual void run (void);

  private:
    /// data plane if we want to access the knowledge base
    madara::knowledge::KnowledgeBase data_;
  };
} // end namespace threads

#endif // _THREAD_${new_thr_uc}_H_
";
        my $source_contents = "
#include \"gams/loggers/GlobalLogger.h\"
#include \"${new_thr}.h\"

namespace knowledge = madara::knowledge;

// constructor
threads::${new_thr}::${new_thr} ()
{
}

// destructor
threads::${new_thr}::~${new_thr} ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::${new_thr}::init (knowledge::KnowledgeBase & knowledge)
{
  // point our data plane to the knowledge base initializing the thread
  data_ = knowledge;
}

/**
 * Executes the actual thread logic. Best practice is to simply do one loop
 * iteration. If you want a long running thread that executes something
 * frequently, see the madara::threads::Threader::runHz method in your
 * controller.
 **/
void
threads::${new_thr}::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    \"threads::${new_thr}::run:\" \
    \" executing\\n\");
}
";
    
        # open files for writing
        open thread_header, ">$threads_path/$new_thr.h" or 
          die "ERROR: Couldn't open $threads_path/$new_thr.h for writing\n";
          print thread_header $header_contents;
        close thread_header;
          
        open thread_source, ">$threads_path/$new_thr.cpp" or 
          die "ERROR: Couldn't open $threads_path/$new_thr.cpp for writing\n";
          print thread_source $source_contents;
        close thread_source;
      }
    }


    foreach my $new_trans (@new_transport)
    {
      if ($new_trans and not -f "$trans_path/$new_trans.h")
      {
        my $new_trans_uc = uc $new_trans;
        my $header_contents;
        my $source_contents;
     
        $header_contents .= "
#ifndef   _TRANSPORT_${new_trans_uc}_H_
#define   _TRANSPORT_${new_trans_uc}_H_

#include \"madara/transport/Transport.h\"
#include \"madara/threads/Threader.h\"
#include \"${new_trans}ReadThread.h\"

namespace transports
{
  /**
   * Custom network transport generated by gpc.pl
   **/
  class ${new_trans} : public madara::transport::Base
  {
  public:
    /**
     * Constructor
     * \@param   id                unique identifier (generally host:port)
     * \@param   new_settings      settings to apply to the transport
     * \@param   context           the knowledge record context
     **/
    ${new_trans} (const std::string & id,
      madara::transport::TransportSettings & new_settings,
      madara::knowledge::KnowledgeBase & context);

    /**
     * Destructor
     **/
    virtual ~${new_trans} ();
    
    /**
     * Sends a list of updates to the domain. This function must be
     * implemented by your transport
     * \@param  modifieds  a list of keys to values of all records that have
     *          been updated and could be sent.
     * \@return  result of operation or -1 if we are shutting down
     **/
    long send_data (const madara::knowledge::VariableReferenceMap & modifieds);
    
  protected:
    /// threads for monitoring knowledge updates
    madara::threads::Threader read_threads_;
  };
}

#endif // _TRANSPORT_${new_trans_uc}_H_
";

        $source_contents .= "
#include <sstream>
#include \"${new_trans}.h\"
#include \"${new_trans}ReadThread.h\"

namespace knowledge = madara::knowledge;

transports::${new_trans}::${new_trans} (
  const std::string & id,
  madara::transport::TransportSettings & new_settings,
  knowledge::KnowledgeBase & knowledge)
: madara::transport::Base (id, new_settings, knowledge.get_context ())
{
  // populate variables like buffer_ based on transport settings
  Base::setup ();
  
  // set the data plane for read threads
  read_threads_.set_data_plane (knowledge);
  
  // check the read hz settings to see if the user has passed something weird
  double hertz = new_settings.read_thread_hertz;
  if (hertz < 0.0)
  {
    hertz = 0.0;
  }

  // create the read threads specified in TransportSettings  
  for (uint32_t i = 0; i < new_settings.read_threads; ++i)
  {
    // construct a unique id for a new thread
    std::stringstream thread_name;
    thread_name << \"read\";
    thread_name << i;
    
    // start the thread at the specified hertz
    read_threads_.run (
      hertz,
      thread_name.str (),
      new ${new_trans}ReadThread (
        id_, new_settings, 
        send_monitor_, receive_monitor_, packet_scheduler_));
  }
}

transports::${new_trans}::~${new_trans} ()
{
}

long
transports::${new_trans}::send_data (
  const madara::knowledge::VariableReferenceMap & modifieds)
{
  /**
   * Return number of bytes sent or negative for error
   **/
  long result (0);
  
  /**
   * This is where you should do your custom transport sending logic/actions
   **/
  
  return result;
}
";   

        # open files for writing
        open transport_header, ">$trans_path/$new_trans.h" or 
          die "ERROR: Couldn't open $trans_path/$new_trans.h for writing\n";
          print transport_header $header_contents;
        close transport_header;
          
        open transport_source, ">$trans_path/$new_trans.cpp" or 
          die "ERROR: Couldn't open $trans_path/$new_trans.cpp for writing\n";
          print transport_source $source_contents;
        close transport_source;
        
        # now, create the read thread class

        $header_contents = "
#ifndef   _TRANSPORT_${new_trans_uc}READTHREAD_H_
#define   _TRANSPORT_${new_trans_uc}READTHREAD_H_

#include <string>

#include \"madara/threads/BaseThread.h\"

namespace transports
{
  /**
  * A custom read thread generated by gpc.pl
  **/
  class ${new_trans}ReadThread : public madara::threads::BaseThread
  {
  public:
    /**
     * Default constructor
     **/
    ${new_trans}ReadThread (
      const std::string & id,
      const madara::transport::TransportSettings & settings,
      madara::transport::BandwidthMonitor & send_monitor,
      madara::transport::BandwidthMonitor & receive_monitor,
      madara::transport::PacketScheduler & packet_scheduler);
    
    /**
     * Destructor
     **/
    virtual ~${new_trans}ReadThread ();
    
    /**
      * Initializes thread with MADARA context
      * \@param   context   context for querying current program state
      **/
    virtual void init (madara::knowledge::KnowledgeBase & knowledge);

    /**
      * Executes the main thread logic
      **/
    virtual void run (void);

  private:
    /// data plane if we want to access the knowledge base
    madara::knowledge::ThreadSafeContext * context_;
    
    /// the unique id of this agent (probably a host:port pairing)
    const std::string id_;
    
    /// the transport settings being used
    madara::transport::QoSTransportSettings settings_;
    
    /// buffer for sending
    madara::utility::ScopedArray <char>      buffer_;

    /// data received rules, defined in Transport settings
    madara::knowledge::CompiledExpression  on_data_received_;
      
    /// monitor the bandwidth used for sending
    madara::transport::BandwidthMonitor & send_monitor_;
    
    /// monitor the bandwidth used by others
    madara::transport::BandwidthMonitor & receive_monitor_;
    
    /// a specialty packet scheduler for experimental drop policies
    madara::transport::PacketScheduler & packet_scheduler_;
  };
} // end namespace threads

#endif // _TRANSPORT_${new_trans_uc}READTHREAD_H_
";
        $source_contents = "
#include \"gams/loggers/GlobalLogger.h\"
#include \"${new_trans}ReadThread.h\"

namespace knowledge = madara::knowledge;

// constructor
transports::${new_trans}ReadThread::${new_trans}ReadThread (
  const std::string & id,
  const madara::transport::TransportSettings & settings,
  madara::transport::BandwidthMonitor & send_monitor,
  madara::transport::BandwidthMonitor & receive_monitor,
  madara::transport::PacketScheduler & packet_scheduler)
: send_monitor_ (send_monitor),
  receive_monitor_ (receive_monitor),
  packet_scheduler_ (packet_scheduler)
{
}

// destructor
transports::${new_trans}ReadThread::~${new_trans}ReadThread ()
{
}

/**
 * Initialization to a knowledge base.
 **/
void
transports::${new_trans}ReadThread::init (knowledge::KnowledgeBase & knowledge)
{
  // grab the context so we have access to update_from_external  
  context_ = &(knowledge.get_context ());
  
  // setup the receive buffer
  if (settings_.queue_length > 0)
    buffer_ = new char [settings_.queue_length];
}

/**
 * Executes the actual thread logic.
 **/
void
transports::${new_trans}ReadThread::run (void)
{
  // prefix for logging purposes
  char * print_prefix = \"transports::${new_trans}ReadThread\";

  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    \"%s::run: executing\\n\", print_prefix);

  /**
   * this should store the number of bytes read into the buffer after your
   * network, socket, serial port or shared memory read
   **/
  uint32_t bytes_read = 0;

  // header for buffer
  madara::transport::MessageHeader * header = 0;

  /**
   * remote host identifier, often available from socket recvs. This 
   * information can be used with trusted/banned host lists in the
   * process_received_update function later. You would have to fill
   * this in with info from whatever transport you are using for this
   * to be effective.
   **/
  char * remote_host = \"\";
  
  // will be filled with rebroadcast records after applying filters
  knowledge::KnowledgeMap rebroadcast_records;

  /**
   * Calls filters on buffered data, updates throughput calculations
   **/
  process_received_update (buffer_.get_ptr (), bytes_read, id_, *context_,
    settings_, send_monitor_, receive_monitor_, rebroadcast_records,
    on_data_received_, print_prefix, remote_host, header);
    
  /**
   * The next run of this method will be determined by read_thread_hertz
   * in the QoSTransportSettings class that is passed in.
   **/
}
";

        # open files for writing
        open transport_header, ">$trans_path/${new_trans}ReadThread.h" or 
          die "ERROR: Couldn't open $trans_path/${new_trans}ReadThread.h for writing\n";
          print transport_header $header_contents;
        close transport_header;
          
        open transport_source, ">$trans_path/${new_trans}ReadThread.cpp" or 
          die "ERROR: Couldn't open $trans_path/${new_trans}ReadThread.cpp for writing\n";
          print transport_source $source_contents;
        close transport_source;
           
        
      } #end if the new transport does not exist
    } #end foreach new transport
      
    
    foreach my $filter (@new_receive_filters)
    {
      if ($filter and not -f "$filters_path/$filter.h")
      {
        my $new_filter_uc = uc $filter;
      
        if ($verbose)
        {
          print ("Adding infrastructure for filter $filter...\n");
        }
         
        # Create file contents for custom filters
       
        my $header_contents = "
#ifndef   _FILTER_${new_filter_uc}_H_
#define   _FILTER_${new_filter_uc}_H_

#include <string>

#include \"madara/filters/AggregateFilter.h\"

namespace filters
{
  /**
  * A stateful filter generated by gpc.pl
  **/
  class $filter : public madara::filters::AggregateFilter
  {
  public:
    /**
     * Constructor
     **/
    $filter ();

    /**
     * Destructor
     **/
    virtual ~$filter ();

    /**
     * The method that filters incoming or outgoing 
     * \@param   records           the aggregated packet being evaluated
     * \@param   transport_context context for querying transport state
     * \@param   vars              context for querying current program state
     **/
    virtual void filter (madara::knowledge::KnowledgeMap & records,
      const madara::transport::TransportContext & transport_context,
      madara::knowledge::Variables & vars);
      
  protected:
    /**
     * any variables needed for filtering decisions can go here. Note that
     * the knowledge base will be provided via the filter method's Variables
     * input.
     **/
  };

} // end filters namespace

#endif // _FILTER_${filter}_H_
";
      
        my $source_contents = " 
#include \"${filter}.h\"

filters::${filter}::${filter} ()
{
}

filters::${filter}::~${filter} ()
{
}

void
filters::${filter}::filter (
  madara::knowledge::KnowledgeMap & records,
  const madara::transport::TransportContext & transport_context,
  madara::knowledge::Variables & vars)
{
  // this function will contain the logic needed for filtering knowledge
}

";
           
        # open files for writing
        open filter_header, ">$filters_path/$filter.h" or 
          die "ERROR: Couldn't open $filters_path/$filter.h for writing\n";
          print filter_header $header_contents;
        close filter_header;
          
        open filter_source, ">$filters_path/$filter.cpp" or 
          die "ERROR: Couldn't open $filters_path/$filter.cpp for writing\n";
          print filter_source $source_contents;
        close filter_source;
          
      } # end if new filter and the source files don't exist
    } #end foreach new receive filter
    
    
    foreach my $filter (@new_send_filters)
    {
      if ($filter and not -f "$filters_path/$filter.h")
      {
        my $new_filter_uc = uc $filter;
      
        if ($verbose)
        {
          print ("Adding infrastructure for filter $filter...\n");
        }
         
        # Create file contents for custom filters
       
        my $header_contents = "
#ifndef   _FILTER_${new_filter_uc}_H_
#define   _FILTER_${new_filter_uc}_H_

#include <string>

#include \"madara/filters/AggregateFilter.h\"

namespace filters
{
  /**
  * A stateful filter generated by gpc.pl
  **/
  class $filter : public madara::filters::AggregateFilter
  {
  public:
    /**
     * Constructor
     **/
    $filter ();

    /**
     * Destructor
     **/
    virtual ~$filter ();

    /**
     * The method that filters incoming or outgoing 
     * \@param   records           the aggregated packet being evaluated
     * \@param   transport_context context for querying transport state
     * \@param   vars              context for querying current program state
     **/
    virtual void filter (madara::knowledge::KnowledgeMap & records,
      const madara::transport::TransportContext & transport_context,
      madara::knowledge::Variables & vars);
      
  protected:
    /**
     * any variables needed for filtering decisions can go here. Note that
     * the knowledge base will be provided via the filter method's Variables
     * input.
     **/
  };

} // end filters namespace

#endif // _FILTER_${filter}_H_
";
      
        my $source_contents = " 
#include \"${filter}.h\"

filters::${filter}::${filter} ()
{
}

filters::${filter}::~${filter} ()
{
}

void
filters::${filter}::filter (
  madara::knowledge::KnowledgeMap & records,
  const madara::transport::TransportContext & transport_context,
  madara::knowledge::Variables & vars)
{
  // this function will contain the logic needed for filtering knowledge
}

";
           
        # open files for writing
        open filter_header, ">$filters_path/$filter.h" or 
          die "ERROR: Couldn't open $filters_path/$filter.h for writing\n";
          print filter_header $header_contents;
        close filter_header;
          
        open filter_source, ">$filters_path/$filter.cpp" or 
          die "ERROR: Couldn't open $filters_path/$filter.cpp for writing\n";
          print filter_source $source_contents;
        close filter_source;
          
      } # end if new filter and the source files don't exist
    } #end foreach new receive filter
    
    
    # get a list of all algorithms
    @algorithms = glob "$algs_path/*.cpp";
    for (my $i = 0; $i < scalar @algorithms; ++$i)
    {
      my ($file, $dir, $suffix) = fileparse($algorithms[$i], qr/\.[^.]*/);
      $algorithms[$i] = $file;
    }
    
    # get a list of all custom platforms
    @platforms = glob "$plats_path/*.cpp";
    for (my $i = 0; $i < scalar @platforms; ++$i)
    {
      my ($file, $dir, $suffix) = fileparse($platforms[$i], qr/\.[^.]*/);
      $platforms[$i] = $file;
    }
    
    # get a list of all custom platform threads
    @platform_threads = glob "$plats_threads_path/*.cpp";
    for (my $i = 0; $i < scalar @platform_threads; ++$i)
    {
      my ($file, $dir, $suffix) = fileparse($platform_threads[$i], qr/\.[^.]*/);
      $platform_threads[$i] = $file;
    }
    
    # get a list of all custom threads
    @threads = glob "$threads_path/*.cpp";
    for (my $i = 0; $i < scalar @threads; ++$i)
    {
      my ($file, $dir, $suffix) = fileparse($threads[$i], qr/\.[^.]*/);
      $threads[$i] = $file;
    }
    
    # get a list of all custom threads
    @transports = glob "$trans_path/*ReadThread.cpp";
    for (my $i = 0; $i < scalar @transports; ++$i)
    {
      my ($file, $dir, $suffix) = fileparse($transports[$i], qr/ReadThread\.[^.]*/);
      $transports[$i] = $file;
    }
    
    if ($verbose)
    {
      print "  Custom algorithms in $algs_path:\n";
      for (0..$#algorithms)
      {
        print "    " . $algorithms[$_] . "\n";
      }
      
      print "  Custom platforms in $plats_path:\n";
      for (0..$#platforms)
      {
        print "    " . $platforms[$_] . "\n";
      }
      
      print "  Custom platform threads in $plats_threads_path:\n";
      for (0..$#platform_threads)
      {
        print "    " . $platform_threads[$_] . "\n";
      }
      
      print "  Custom threads in $threads_path:\n";
      for (0..$#threads)
      {
        print "    " . $threads[$_] . "\n";
      }
      
      print "  Custom transports in $trans_path:\n";
      for (0..$#transports)
      {
        print "    " . $transports[$_] . "\n";
      }
    }
    
    if (not -f "$src_path/controller.cpp")
    {
      my $run_contents;
    
      if ($verbose)
      {
        print ("  Making changes to $sim_path/run.pl...\n");
      }
           
      # open files for reading 
      open run_file, "$sim_path/run.pl" or
        die "ERROR: Couldn't open $sim_path/run.pl\n"; 
        $run_contents = join("", <run_file>); 
      close run_file;
      
      # replace the gams_controller with a custom controller
      if ($run_contents =~ s/controller\s*=\s*\"[^\"]+\"/controller = \"\$dir\/..\/bin\/custom_controller\"/)
      {
        if ($verbose)
        {
          print ("  Replaced controller with \$dir/bin/custom_controller\n");
        }
      }
      else
      {
        if ($verbose)
        {
          print ("  Unable to replace controller with \$dir/bin/custom_controller\n");
        }
      }
  
      if ($verbose)
      {
        print ("  Writing updates to $sim_path/run.pl...\n");
      }

      # write changes to simulation run script     
      open run_file, ">$sim_path/run.pl" or
        die "ERROR: Couldn't open $sim_path/run.pl\n"; 
        print run_file $run_contents;
      close run_file;   

      my $project_contents = "
project (custom_controller) : using_gams, using_madara, using_vrep {
  exeout = bin
  exename = custom_controller

  macros +=  _USE_MATH_DEFINES

  Documentation_Files {
    README.txt
  }

  Header_Files {
    src/
    src/algorithms
    src/algorithms/threads
    src/containers
    src/filters
    src/platforms
    src/platforms/threads
    src/threads
    src/transports
  }

  Source_Files {
    src
    src/algorithms
    src/algorithms/threads
    src/containers
    src/filters
    src/platforms
    src/platforms/threads
    src/threads
    src/transports
  }
}
";

   
      my $workspace_contents = "
workspace {
  specific(make) {
    cmdline += -value_template ccflags+=-std=c++11
    cmdline += -value_template compile_flags+=-Wextra
    cmdline += -value_template compile_flags+=-pedantic
  }

  specific(gnuace) {
    cmdline += -value_template \"compile_flags+=-std=c++11 -g -Og\"
    cmdline += -value_template compile_flags+=-Wextra
    cmdline += -value_template compile_flags+=-pedantic
  }

  project.mpc
  docs/Documentation.mpc
}

";

      open project_file, ">$path/project.mpc" or 
        die "ERROR: Couldn't open $path/project.mpc for writing\n";
        print project_file $project_contents;
      close project_file;

      open workspace_file, ">$path/workspace.mwc" or 
        die "ERROR: Couldn't open $path/workspace.mwc for writing\n";
        print workspace_file $workspace_contents;
      close workspace_file;
    }

    if (not $thread_hz)
    {
      $thread_hz = 1.0;
    }

    if (not -f "$src_path/controller.cpp")
    {
      my $controller_contents = "

#include \"madara/knowledge/KnowledgeBase.h\"
#include \"madara/threads/Threader.h\"
#include \"gams/controllers/BaseController.h\"
#include \"gams/loggers/GlobalLogger.h\"

// DO NOT DELETE THIS SECTION

// begin algorithm includes\n";

      for my $new_alg (@new_algorithm)
      {
        $controller_contents .= "#include \"algorithms/$new_alg.h\"\n";
      }

      $controller_contents .= "// end algorithm includes

// begin platform includes\n";

      for my $new_plat (@new_platform)
      {
        $controller_contents .= "#include \"platforms/$new_plat.h\"\n";
      }

      $controller_contents .= "// end platform includes

// begin thread includes\n";

      for my $new_thr (@new_thread)
      {
        $controller_contents .= "#include \"threads/$new_thr.h\"\n";
      }

      $controller_contents .= "// end thread includes

// begin transport includes\n";

      for my $new_trans (@new_transport)
      {
        $controller_contents .= "#include \"transports/$new_trans.h\"\n";
      }

      $controller_contents .= "// end transport includes

// begin filter includes\n";

      for my $filter (@new_receive_filters)
      {
        $controller_contents .= "#include \"filters/$filter.h\"\n";
      }

      for my $filter (@new_send_filters)
      {
        $controller_contents .= "#include \"filters/$filter.h\"\n";
      }

      $controller_contents .= "// end filter includes

// END DO NOT DELETE THIS SECTION

const std::string default_broadcast (\"192.168.1.255:15000\");
// default transport settings
std::string host (\"\");
const std::string default_multicast (\"239.255.0.1:4150\");
madara::transport::QoSTransportSettings settings;

// create shortcuts to MADARA classes and namespaces
namespace controllers = gams::controllers;
typedef madara::knowledge::KnowledgeRecord   Record;
typedef Record::Integer Integer;

const std::string KNOWLEDGE_BASE_PLATFORM_KEY (\".platform\");
bool plat_set = false;
std::string platform (\"debug\");
std::string algorithm (\"debug\");
std::vector <std::string> accents;

// madara commands from a file
std::string madara_commands = \"\";

// for setting debug levels through command line
int madara_debug_level (-1);
int gams_debug_level (-1);

// number of agents in the swarm
Integer num_agents (-1);

// file path to save received files to
std::string file_path;

// filename to save transport settings to
std::string save_transport;
std::string save_transport_prefix;
std::string save_transport_text;
std::string load_transport_prefix;

// controller settings for controller configuration
gams::controllers::ControllerSettings controller_settings;

void print_usage (char * prog_name)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_ALWAYS,
\"\\nProgram summary for %s:\\n\\n\" \
\"     Loop controller setup for gams\\n\" \
\" [-A |--algorithm type]        algorithm to start with\\n\" \
\" [-a |--accent type]           accent algorithm to start with\\n\" \
\" [-b |--broadcast ip:port]     the broadcast ip to send and listen to\\n\" \
\" [--checkpoint-on-loop]        save checkpoint after each control loop\\n\" \
\" [--checkpoint-on-send]        save checkpoint before send of updates\\n\" \
\" [-c |--checkpoint prefix]     the filename prefix for checkpointing\\n\" \
\" [-d |--domain domain]         the knowledge domain to send and listen to\\n\" \
\" [-e |--rebroadcasts num]      number of hops for rebroadcasting messages\\n\" \
\" [-f |--logfile file]          log to a file\\n\" \
\" [-i |--id id]                 the id of this agent (should be non-negative)\\n\" \
\" [-lt|--load-transport file] a file to load transport settings from\\n\" \
\" [-ltp|--load-transport-prefix prfx] prefix of saved settings\\n\" \
\" [-ltt|--load-transport-text file] a text file to load transport settings from\\n\" \
\" [--madara-level level]        the MADARA logger level (0+, higher is higher detail)\\n\" \
\" [--gams-level level]          the GAMS logger level (0+, higher is higher detail)\\n\" \
\" [--loop-hertz hz]             hertz to run the MAPE loop\\n\"\
\" [-L |--loop-time time]        time to execute loop\\n\"\
\" [-m |--multicast ip:port]     the multicast ip to send and listen to\\n\" \
\" [-M |--madara-file <file>]    file containing madara commands to execute\\n\" \
\"                               multiple space-delimited files can be used\\n\" \
\" [-n |--num_agents <number>]   the number of agents in the swarm\\n\" \
\" [-o |--host hostname]         the hostname of this process (def:localhost)\\n\" \
\" [-p |--platform type]         platform for loop (vrep, dronerk)\\n\" \
\" [-P |--period period]         time, in seconds, between control loop executions\\n\" \
\" [-q |--queue-length length]   length of transport queue in bytes\\n\" \
\" [-r |--reduced]               use the reduced message header\\n\" \
\" [-s |--send-hertz hertz]      send hertz rate for modifications\\n\" \
\" [-st|--save-transport file] a file to save transport settings to\\n\" \
\" [-stp|--save-transport-prefix prfx] prefix to save settings at\\n\" \
\" [-stt|--save-transport-text file] a text file to save transport settings to\\n\" \
\" [-t |--target path]           file system location to save received files (NYI)\\n\" \
\" [-u |--udp ip:port]           a udp ip to send to (first is self to bind to)\\n\" \
\" [--zmq proto:ip:port]         specifies a 0MQ transport endpoint\\n\"
\"\\n\",
        prog_name);
  exit (0);
}

// handle command line arguments
void handle_arguments (int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i)
  {
    std::string arg1 (argv[i]);

    if (arg1 == \"-A\" || arg1 == \"--algorithm\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        algorithm = argv[i + 1];
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"-a\" || arg1 == \"--accent\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        accents.push_back (argv[i + 1]);
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"-b\" || arg1 == \"--broadcast\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = madara::transport::BROADCAST;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"-c\" || arg1 == \"--checkpoint\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        controller_settings.checkpoint_prefix = argv[i + 1];
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"--checkpoint-on-loop\")
    {
      controller_settings.checkpoint_strategy =
        gams::controllers::CHECKPOINT_EVERY_LOOP;
    }
    else if (arg1 == \"--checkpoint-on-send\")
    {
      controller_settings.checkpoint_strategy =
        gams::controllers::CHECKPOINT_EVERY_SEND;
    }
    else if (arg1 == \"-d\" || arg1 == \"--domain\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        settings.write_domain = argv[i + 1];
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"-e\" || arg1 == \"--rebroadcasts\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        int hops;
        std::stringstream buffer (argv[i + 1]);
        buffer >> hops;

        settings.set_rebroadcast_ttl (hops);
        settings.enable_participant_ttl (hops);
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"-f\" || arg1 == \"--logfile\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        madara::logger::global_logger->add_file (argv[i + 1]);
        gams::loggers::global_logger->add_file (argv[i + 1]);
      }
      else
        print_usage (argv[0]);

      ++i;
    }

    else if (arg1 == \"-i\" || arg1 == \"--id\")
    {
      if (i + 1 < argc && argv[i +1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> settings.id;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"-lt\" || arg1 == \"--load-transport\")
    {
      if (i + 1 < argc)
      {
        if (load_transport_prefix == \"\")
          settings.load (argv[i + 1]);
        else
          settings.load (argv[i + 1], load_transport_prefix);
      }

      ++i;
    }
    else if (arg1 == \"-ltp\" || arg1 == \"--load-transport-prefix\")
    {
      if (i + 1 < argc)
      {
        load_transport_prefix = argv[i + 1];
      }

      ++i;
    }
    else if (arg1 == \"-ltt\" || arg1 == \"--load-transport-text\")
    {
      if (i + 1 < argc)
      {
        if (load_transport_prefix == \"\")
          settings.load_text (argv[i + 1]);
        else
          settings.load_text (argv[i + 1], load_transport_prefix);
      }

      ++i;
    }
    else if (arg1 == \"--madara-level\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> madara_debug_level;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"--gams-level\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> gams_debug_level;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"-L\" || arg1 == \"--loop-time\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> controller_settings.run_time;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"--loop-hertz\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> controller_settings.loop_hertz;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"-m\" || arg1 == \"--multicast\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = madara::transport::MULTICAST;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"-M\" || arg1 == \"--madara-file\")
    {
      bool files = false;
      ++i;
      for (;i < argc && argv[i][0] != '-'; ++i)
      {
        std::string filename = argv[i];
        if (madara::utility::file_exists (filename))
        {
          madara_commands += madara::utility::file_to_string (filename);
          madara_commands += \";\\n\";
          files = true;
        }
      }
      --i;

      if (!files)
        print_usage (argv[0]);
    }
    else if (arg1 == \"-n\" || arg1 == \"--num_agents\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> num_agents;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"-o\" || arg1 == \"--host\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        host = argv[i + 1];
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"-p\" || arg1 == \"--platform\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        platform = argv[i + 1];
        plat_set = true;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"-P\" || arg1 == \"--period\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> controller_settings.loop_hertz;

        controller_settings.loop_hertz = 1 / controller_settings.loop_hertz;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"-q\" || arg1 == \"--queue-length\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> settings.queue_length;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"-r\" || arg1 == \"--reduced\")
    {
      settings.send_reduced_message_header = true;
    }
    else if (arg1 == \"-t\" || arg1 == \"--target\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        file_path = argv[i + 1];
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"-st\" || arg1 == \"--save-transport\")
    {
      if (i + 1 < argc)
      {
        save_transport = argv[i + 1];
      }

      ++i;
    }
    else if (arg1 == \"-stp\" || arg1 == \"--save-transport-prefix\")
    {
      if (i + 1 < argc)
      {
        save_transport_prefix = argv[i + 1];
      }

      ++i;
    }
    else if (arg1 == \"-stt\" || arg1 == \"--save-transport-text\")
    {
      if (i + 1 < argc)
      {
        save_transport_text = argv[i + 1];
      }

      ++i;
    }
    else if (arg1 == \"-u\" || arg1 == \"--udp\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = madara::transport::UDP;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == \"--zmq\")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = madara::transport::ZMQ;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else
    {
      print_usage (argv[0]);
    }
  }
}

// perform main logic of program
int main (int argc, char ** argv)
{
  settings.type = madara::transport::MULTICAST;
 
  // handle all user arguments
  handle_arguments (argc, argv);

  if (settings.hosts.size () == 0)
  {
    // setup default transport as multicast
    settings.hosts.resize (1);
    settings.hosts[0] = default_multicast;
  }
  
  // set this once to allow for debugging knowledge base creation
  if (madara_debug_level >= 0)
  {
    madara::logger::global_logger->set_level (madara_debug_level);
  }

  // save transport always happens after all possible transport chagnes
  if (save_transport != \"\")
  {
    if (save_transport_prefix == \"\")
      settings.save (save_transport);
    else
      settings.save (save_transport, save_transport_prefix);
  }

  // save transport always happens after all possible transport chagnes
  if (save_transport_text != \"\")
  {
    if (save_transport_prefix == \"\")
      settings.save_text (save_transport_text);
    else
      settings.save_text (save_transport_text, save_transport_prefix);
  }

  // create knowledge base and a control loop
  madara::knowledge::KnowledgeBase knowledge;
  
  // begin on receive filters";
  
  foreach my $filter (@new_receive_filters)
  {
    $controller_contents .= "
  settings.add_receive_filter (new filters::$filter ());";
  }
  
    $controller_contents .= "
  // end on receive filters
  
  // begin on send filters";
  
  foreach my $filter (@new_send_filters)
  {
    $controller_contents .= "
  settings.add_send_filter (new filters::$filter ());";
  }
  
    $controller_contents .= "
  // end on send filters
  
  // if you only want to use custom transports, delete following
  knowledge.attach_transport (host, settings);
  
  // begin transport creation ";
  
  foreach my $new_trans (@new_transport)
  {
    $controller_contents .= "
  // add $new_trans factory
  knowledge.attach_transport (new transports::$new_trans (
    knowledge.get_id (), settings, knowledge));";
  }
  
    $controller_contents .= "
  // end transport creation
  
  // set this once to allow for debugging controller creation
  if (gams_debug_level >= 0)
  {
    gams::loggers::global_logger->set_level (gams_debug_level);
  }

  controllers::BaseController controller (knowledge, controller_settings);
  madara::threads::Threader threader (knowledge);

  // initialize variables and function stubs
  controller.init_vars (settings.id, num_agents);
  
  std::vector <std::string> aliases;
  
  // begin adding custom algorithm factories";
  
  foreach my $new_alg (@new_algorithm)
  {
    $controller_contents .= "
  // add $new_alg factory
  aliases.clear ();
  aliases.push_back (\"$new_alg\");
  controller.add_algorithm_factory (aliases,
    new algorithms::${new_alg}Factory ());\n";
  }

    $controller_contents .= "
  // end adding custom algorithm factories

  // begin adding custom platform factories";

  foreach my $new_plat (@new_platform)
  {
    $controller_contents .= "
  // add $new_plat factory
  aliases.clear ();
  aliases.push_back (\"$new_plat\");
  controller.add_platform_factory (aliases,
    new platforms::${new_plat}Factory ());\n";
  }
  
    $controller_contents .= "
  // end adding custom platform factories
  
  // read madara initialization
  if (madara_commands != \"\")
  {
    knowledge.evaluate (madara_commands,
      madara::knowledge::EvalSettings(false, true));
  }
  
  // set debug levels if they have been set through command line
  if (madara_debug_level >= 0)
  {
    std::stringstream temp_buffer;
    temp_buffer << \"agent.\" << settings.id << \".madara_debug_level = \";
    temp_buffer << madara_debug_level;

    madara::logger::global_logger->set_level (madara_debug_level);

    // modify the debug level being used but don't send out to others
    knowledge.evaluate (temp_buffer.str (),
      madara::knowledge::EvalSettings (true, true));
  }

  if (gams_debug_level >= 0)
  {
    std::stringstream temp_buffer;
    temp_buffer << \"agent.\" << settings.id << \".gams_debug_level = \";
    temp_buffer << gams_debug_level;

    gams::loggers::global_logger->set_level (gams_debug_level);

    // modify the debug level being used but don't send out to others
    knowledge.evaluate (temp_buffer.str (),
      madara::knowledge::EvalSettings (true, true));
  }

  // initialize the platform and algorithm
  // default to platform in knowledge base if platform not set in command line
  if (!plat_set && knowledge.exists (KNOWLEDGE_BASE_PLATFORM_KEY))
    platform = knowledge.get (KNOWLEDGE_BASE_PLATFORM_KEY).to_string ();
  controller.init_platform (platform);
  controller.init_algorithm (algorithm);

  // add any accents
  for (unsigned int i = 0; i < accents.size (); ++i)
  {
    controller.init_accent (accents[i]);
  }

  /**
   * WARNING: the following section will be regenerated whenever new threads
   * are added via this tool. So, you can adjust hertz rates and change how
   * the thread is initialized, but the entire section will be regenerated
   * with all threads in the threads directory, whenever you use the new
   * thread option with the gpc.pl script.
   **/

  // begin thread creation";

  foreach my $new_thr (@new_thread)
  {
    $controller_contents .= "
  threader.run (${thread_hz}, \"${new_thr}\", new threads::${new_thr}());";
  }
  
    $controller_contents .= "
  // end thread creation
  
  /**
   * END WARNING
   **/
  
  // run a mape loop for algorithm and platform control
  controller.run ();

  // terminate all threads after the controller
  threader.terminate ();
  
  // wait for all threads
  threader.wait ();
  
  // print all knowledge values
  knowledge.print ();

  return 0;
}

";
      # write changes to the freshly created files

      if ($verbose)
      {
        print ("  Writing new source and project files...\n");
      }
         
      open controller_file, ">$src_path/controller.cpp" or 
        die "ERROR: Couldn't open $src_path/controller.cpp for writing\n";
        print controller_file $controller_contents;
      close controller_file;
      
    } # end if custom controller does not exist
    else # begin custom controller already exists
    {
      # custom controller already exists so read it in
      
      if ($verbose)
      {
        print ("Updating controller with custom class usage...\n");
      }
         
      my $controller_contents;
      my ($algorithm_includes, $platform_includes,
      $thread_includes, $transport_includes, $receive_filter_includes,
      $send_filter_includes);
      my ($algorithm_creation, $platform_creation,
      $thread_creation, $transport_creation, $receive_filter_creation,
      $send_filter_creation);
      
      # read the controller source file
      open controller_file, "$src_path/controller.cpp" or 
        die "ERROR: Couldn't open $src_path/controller.cpp for writing\n";
        $controller_contents = join("", <controller_file>); 
      close controller_file;
      
      if (scalar @algorithms > 0)
      {
        if ($verbose)
        {
          print ("  Custom algorithms detected. Updating...\n");
        }
       
        for (my $i = 0; $i < scalar @algorithms; ++$i)
        {
          $algorithm_includes .= "\n#include \"algorithms/" .
            $algorithms[$i] . ".h\"";
          $algorithm_creation .= "\n
  // add ${algorithms[$i]} factory
  aliases.clear ();
  aliases.push_back (\"${algorithms[$i]}\");\n
  controller.add_algorithm_factory (aliases,
    new algorithms::${algorithms[$i]}Factory ());";
        } 
       
        if ($verbose)
        {
          print ("  Custom algorithms detected. Updating...\n");
        }
        
        # change the includes         
    $controller_contents =~
      s/\/\/ begin algorithm includes(.|\s)*\/\/ end algorithm includes/\/\/ begin algorithm includes${algorithm_includes}\n\/\/ end algorithm includes/;
        
        # change the creation process
    $controller_contents =~
      s/\/\/ begin adding custom algorithm factories(.|\s)*\/\/ end adding custom algorithm factories/\/\/ begin adding custom algorithm factories${algorithm_creation}\n  \/\/ end adding custom algorithm factories/;
      } # end if there are custom algorithms

      if (scalar @platforms > 0)
      {
        if ($verbose)
        {
          print ("  Custom platforms detected. Updating...\n");
        }
       
        for (my $i = 0; $i < scalar @platforms; ++$i)
        {
          $platform_includes .= "\n#include \"platforms/" .
            $platforms[$i] . ".h\"";
          
          # insert extra space in between platform creations
          $platform_creation .= "\n
  // add ${platforms[$i]} factory
  aliases.clear ();
  aliases.push_back (\"${platforms[$i]}\");\n
  controller.add_platform_factory (aliases,
    new platforms::${platforms[$i]}Factory ());";
        } 
        
        # change the includes         
    $controller_contents =~
      s/\/\/ begin platform includes(.|\s)*\/\/ end platform includes/\/\/ begin platform includes${platform_includes}\n\/\/ end platform includes/;
        
        # change the creation process
    $controller_contents =~
      s/\/\/ begin adding custom platform factories(.|\s)*\/\/ end adding custom platform factories/\/\/ begin adding custom platform factories${platform_creation}\n  \/\/ end adding custom platform factories/;
      } # end if there are custom platforms

      if (scalar @threads > 0)
      {
        if ($verbose)
        {
          print ("  Custom threads detected. Updating...\n");
        }
       
        for (my $i = 0; $i < scalar @threads; ++$i)
        {
          $thread_includes .= "\n#include \"threads/" .
            $threads[$i] . ".h\"";            
          $thread_creation .= "
  threader.run (${thread_hz}, \"${threads[$i]}\", new threads::${threads[$i]} ());";
        }
        
        # change the includes         
    $controller_contents =~
      s/\/\/ begin thread includes(.|\s)*\/\/ end thread includes/\/\/ begin thread includes${thread_includes}\n\/\/ end thread includes/;
        
        # change the creation process
    $controller_contents =~
      s/\/\/ begin thread creation(.|\s)*\/\/ end thread creation/\/\/ begin thread creation${thread_creation}\n  \/\/ end thread creation/;
      } # end if there are custom threads
      
      if (scalar @transports > 0)
      {
        if ($verbose)
        {
          print ("  Custom transports detected. Updating...\n");
        }
       
        for (my $i = 0; $i < scalar @transports; ++$i)
        {
          $transport_includes .= "\n#include \"transports/" .
            $transports[$i] . ".h\"";            
          $transport_creation .= "
  knowledge.attach_transport (new transports::${transports[$i]} (
    knowledge.get_id (), settings, knowledge));";
        }
        
        # change the includes         
    $controller_contents =~
      s/\/\/ begin transport includes(.|\s)*\/\/ end transport includes/\/\/ begin transport includes${transport_includes}\n\/\/ end transport includes/;
        
        # change the creation process
    $controller_contents =~
      s/\/\/ begin transport creation(.|\s)*\/\/ end transport creation/\/\/ begin transport creation${transport_creation}\n  \/\/ end transport creation/;
      } # end if there are custom threads
      
      if (scalar @new_receive_filters > 0)
      {
        if ($verbose)
        {
          print ("  Custom receive filters detected. Updating...\n");
        }
       
        if (not $controller_contents =~ /\/\/ end filter includes/)
        {
          $controller_contents =~
      s/\/\/ end transport includes/\/\/ end transport includes\n\n\/\/ begin filter includes\n\/\/ end filter includes/;
        }
       
        if (not $controller_contents =~ /\/\/ end on receive filters/)
        {
          $controller_contents =~
      s/KnowledgeBase knowledge;/KnowledgeBase knowledge;\n\n  \/\/ begin on receive filters\n  \/\/ end on receive filters/;
        }
       
        foreach my $filter (@new_receive_filters)
        {
          if (not $controller_contents =~ /#include \"filters\/${filter}\.h\"/)
          {
            $receive_filter_includes .= "\n#include \"filters/${filter}.h\"";
          }
          $receive_filter_creation .= "
  settings.add_receive_filter (new filters::${filter} ());";
        }
        
        if ($receive_filter_includes)
        {
          # change the includes         
          $controller_contents =~
      s/[\n \r]+\/\/ end filter includes/${receive_filter_includes}\n\/\/ end filter includes/;
        }
        # change the creation process
        $controller_contents =~
      s/[\n \r]+\/\/ end on receive filters/${receive_filter_creation}\n  \/\/ end on receive filters/;
    
      } # end if there are custom read filters
      
      
      if (scalar @new_send_filters > 0)
      {
        if ($verbose)
        {
          print ("  Custom send filters detected. Updating...\n");
        }
       
        if (not $controller_contents =~ /\/\/ end filter includes/)
        {
        $controller_contents =~
      s/\/\/ end transport includes/\/\/ end transport includes\n\n\/\/ begin filter includes\n\/\/ end filter includes/;
        }
       
        if (not $controller_contents =~ /\/\/ end on send filters/)
        {
        $controller_contents =~
      s/KnowledgeBase knowledge;/KnowledgeBase knowledge;\n\n  \/\/ begin on send filters\n  \/\/ end on send filters/;
        }
       
        foreach my $filter (@new_send_filters)
        {
          if (not $controller_contents =~ /#include \"filters\/${filter}\.h\"/)
          {
            $send_filter_includes .= "\n#include \"filters/${filter}.h\"";
          }
          $send_filter_creation .= "
  settings.add_send_filter (new filters::${filter} ());";
        }
        
        if ($send_filter_includes)
        {
          # change the includes         
          $controller_contents =~
      s/[\n \r]+\/\/ end filter includes/${send_filter_includes}\n\/\/ end filter includes/;
        }
        
        # change the creation process
        $controller_contents =~
      s/[\n \r]+\/\/ end on send filters/${send_filter_creation}\n  \/\/ end on send filters/;
    
      } # end if there are custom read filters
      
      
      if ($verbose)
      {
        print ("  Writing updates to $src_path/controller.cpp...\n");
      }
        
      # write the controller source file
      open controller_file, ">$src_path/controller.cpp" or 
        die "ERROR: Couldn't open $src_path/controller.cpp for writing\n";
        print controller_file $controller_contents;
      close controller_file;
    
    } #end if custom controller already existed

    
  } # end if new algorithm or new platform or new thread
  
  
  # copy appropriate script to the workspace
  if (not -f "$path/action.sh" or not -f "$path/action.bat")
  {
    copy "$script_dir/windows/action.bat", "$path/";
    copy "$script_dir/linux/action.sh", "$path/";
    chmod 0755, "$path/action.sh";
  }
  
  # copy documentation autogeneration files
  if (not -f "$path/docs/Documentation.mpc")
  {
    copy "$script_dir/common/docs/Documentation.mpc", "$path/docs/";
  }

  if (not -f "$path/docs/Doxyfile.dxy")
  {
    copy "$script_dir/common/docs/Doxyfile.dxy", "$path/docs/";
  }

  if (not -f "$path/docs/get_version.pl")
  {
    copy "$script_dir/common/docs/get_version.pl", "$path/docs/";
    chmod 0755, "$path/docs/get_version.pl";
  }

  if (not -f "$path/docs/MainPage.md")
  {
    copy "$script_dir/common/docs/MainPage.md", "$path/docs/";
  }

  # copy src directory autogeneration files
  if (not -f "$path/src/Namespaces.h")
  {
    copy "$script_dir/common/src/Namespaces.h", "$path/src";
  }

  # copy main directory autogeneration files
  if (not -f "$path/doxygen_help_gen.mpb")
  {
    copy "$script_dir/common/doxygen_help_gen.mpb", "$path/";
  }

  if (not -f "$path/using_boost.mpb")
  {
    copy "$script_dir/common/using_boost.mpb", "$path/";
  }

  if (not -f "$path/using_gams.mpb")
  {
    copy "$script_dir/common/using_gams.mpb", "$path/";
  }

  if (not -f "$path/using_madara.mpb")
  {
    copy "$script_dir/common/using_madara.mpb", "$path/";
  }

  if (not -f "$path/VERSION.txt")
  {
    copy "$script_dir/common/VERSION.txt", "$path/";
  }

  if (not -f "$path/README.txt")
  {
    my @custom_type_list = ();
    my $readme_contents = "
INTRO:

  This directory has been created by the gpc.pl script and contains
  a custom simulation";
  
    if (scalar @algorithms > 0)
    {
      push @custom_type_list, "algorithms";
    }
  
    if (scalar @platforms > 0)
    {
      push @custom_type_list, "platforms";
    }
  
    if (scalar @threads > 0)
    {
      push @custom_type_list, "threads";
    }
  
    if (scalar @custom_type_list > 0)
    {
      my $i;
      
      unshift @custom_type_list, "GAMS controller";
      for ($i = 0; $i < scalar @custom_type_list - 1; ++$i)
      {
        $readme_contents .= ", ${custom_type_list[$i]}";
      }
      
      $readme_contents .= " and ${custom_type_list[$i]}.";
    }
    else
    {
      $readme_contents .= ".";
    }
  
  
    $readme_contents .= "\n
HOW TO:\n";

    if (scalar @algorithms > 0)
    {
      $readme_contents .= "
  EDIT YOUR ALGORITHMS:
  
    Open " . $algorithms[0] . ".cpp|h with your favorite programming environment / editor.
    Each method in your algorithm should be non-blocking or the call will
    block the controller. It is in your best interest to poll information from
    the environment and knowledge base, rather than blocking on an operating
    system call.\n";
    }

    if (scalar @platforms > 0)
    {
      $readme_contents .= "
  EDIT YOUR PLATFORMS:
  
    Open " . $platforms[0] . ".cpp|h with your favorite programming environment / editor.
    Each method in your platform should be non-blocking or the call will
    block the controller. It is in your best interest to poll information from
    the environment and knowledge base, rather than blocking on an operating
    system call.\n";
    }

    if (scalar @platform_threads > 0)
    {
      $readme_contents .= "
  EDIT YOUR PLATFORM THREADS:
  
    Open " . $platform_threads[0] . ".cpp|h with your favorite programming environment / editor.\n";
    }

    if (scalar @threads > 0)
    {
      $readme_contents .= "
  EDIT YOUR THREADS:
  
    Open " . $threads[0] . ".cpp|h with your favorite programming environment / editor.\n";
    }

    $readme_contents .= "
  COMPILE ON LINUX:
  
    mwc.pl -type gnuace workspace.mwc
    make vrep=1
    
  COMPILE ON WINDOWS:
  
    mwc.pl -type vc12 workspace.mwc 
    
    <Open Visual Studio and compile project>. Note that you should compile the
    solution in the same settings you used for ACE, MADARA, and GAMS. For most
    users, this is probably Release mode for x64. However, whatever you use
    make sure that you have compiled ACE, MADARA, and GAMS in those settings.
    
  RUN THE SIMULATION:
    open VREP simulator
    perl sim/run.pl 
    
";

    open readme_file, ">$path/README.txt" or 
      die "ERROR: Couldn't open $path/README.txt for writing\n";
      print readme_file $readme_contents;
    close readme_file;
  } #end README does not exist
  elsif (scalar @new_algorithm > 0 or scalar @new_platform > 0
         or scalar @new_thread > 0)
  {
    if ($verbose)
    {
      print ("Updating readme file with any custom types...\n");
    }
         
    my $readme_contents;
    my @custom_type_list;
    my $custom_intro;
    my $custom_howto;

    # read the controller source file
    open readme_file, "$path/README.txt" or 
      die "ERROR: Couldn't open $path/README.txt for writing\n";
      $readme_contents = join("", <readme_file>); 
    close readme_file;
  
    # build a list of all the types of custom types being used
    if (scalar @algorithms > 0)
    {
      if ($verbose)
      {
        print ("  Found custom algorithms...\n");
      }
         
      push @custom_type_list, "algorithms";
      $custom_howto .= "
  EDIT YOUR ALGORITHMS:
  
    Open " . $algorithms[0] .
    ".cpp|h with your favorite programming environment / editor.
    Each method in your algorithm should be non-blocking or the call will
    block the controller. It is in your best interest to poll information from
    the environment and knowledge base, rather than blocking on an operating
    system call.\n";
    }
    if (scalar @platforms > 0)
    {
      if ($verbose)
      {
        print ("  Found custom platforms...\n");
      }
         
      push @custom_type_list, "platforms";
      $custom_howto .= "
  EDIT YOUR PLATFORMS:
  
    Open " . $platforms[0] . 
    ".cpp|h with your favorite programming environment / editor.
    Each method in your platform should be non-blocking or the call will
    block the controller. It is in your best interest to poll information from
    the environment and knowledge base, rather than blocking on an operating
    system call.\n";
    }
    if (scalar @threads > 0)
    {
      if ($verbose)
      {
        print ("  Found custom threads...\n");
      }
         
      push @custom_type_list, "threads";
      $custom_howto .= "
  EDIT YOUR THREADS:
  
    Open " . $threads[0] .
      ".cpp|h with your favorite programming environment / editor.\n";
    }
    
    # build a string for the intro referencing the custom elements in diretory
    
    if (scalar @custom_type_list > 0)
    {
      my $i;
      
      unshift @custom_type_list, "GAMS controller";
      for ($i = 0; $i < scalar @custom_type_list - 1; ++$i)
      {
        $custom_intro .= ", ${custom_type_list[$i]}";
      }
      
      $custom_intro .= " and ${custom_type_list[$i]}";
    }
  
    $readme_contents =~
      s/custom simulation[^\.\n]*/custom simulation${custom_intro}/;
  
    $readme_contents =~
      s/HOW TO(.|\s)*COMPILE ON LINUX/HOW TO:\n${custom_howto}\n  COMPILE ON LINUX/;
  
    open readme_file, ">$path/README.txt" or 
      die "ERROR: Couldn't open $path/README.txt for writing\n";
      print readme_file $readme_contents;
    close readme_file;
  } # end readme does exist
} #end !$help

#########################
# Returns true if point is inside polygon, false otherwise
#########################
sub inside_polygon
{
  my ($point_ref, $vertices_ref) = @_;
  my @point = @{$point_ref};
  my @vertices = @{$vertices_ref};
  
  my $num_vertices = scalar @vertices;
  
  my $result = undef;
  my $y = $point[1];
  my $x = $point[0];
  
  for (my $i = 0, my $j = $num_vertices - 1; $i < $num_vertices; $j = $i++)
  {
    my $iy = $vertices[$i][1];
    my $ix = $vertices[$i][0];
    my $jy = $vertices[$j][1];
    my $jx = $vertices[$j][0];
  
    if (
         ((($iy <= $y) and ($y < $jy)) or (($jy <= $y) and ($y < $iy)))
          and
          ($x < ($jx - $ix) * ($y - $iy) / ($jy - $iy) + $ix)
        )
    {
      $result = !$result;
    }
  }
  
  if ($verbose && $result)
  {
    print ("    inside_polygon: SUCCESS: [$x, $y].\n");
  }
  elsif ($verbose)
  {
    print ("    inside_polygon: FAIL: [$x, $y].\n");
  }
  
  return $result;
}
