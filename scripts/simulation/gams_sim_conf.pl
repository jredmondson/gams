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
use POSIX;

my $script = fileparse($0);
my $agents;
my @border = ();
my $broadcast;
my $duration;
my $equidistant;
my $buffer = 5;
my $first;
my $gams_debug;
my $help;
my $hz;
my $invert;
my $last;
my $madara_debug;
my $max_lat;
my $max_lon;
my $min_height;
my $min_lat;
my $min_lon;
my $multicast;
my $height_diff = 1;
my $ordered;
my $path = '.';
my $permute;
my $platform;
my @points;
my $priority;
my $randomize;
my $region;
my $remove_files;
my $remove_logs;
my $rotate;
my $surface;
my @udp;
my $unique;
my $update_vrep;
my $verbose;
my $vrep_start_port;

# setup options parser
GetOptions(
  'agents|a=i' => \$agents,
  'buffer=i' => \$buffer,
  'border=s' => \@border,
  'broadcast|b=s' => \$broadcast,
  'duration|d|t=i' => \$duration,
  'equidistant|distributed' => \$equidistant,
  'first|f=i' => \$first,
  'gams_debug|gd=i' => \$gams_debug,
  'help|h' => \$help,
  'height_diff=i' => \$height_diff,
  'hz|z=i' => \$hz,
  'invert|i' => \$invert,
  'last|l=i' => \$last,
  'madara_debug|md=i' => \$madara_debug,
  'max_lat|max-lat=f' => \$max_lat,
  'max_lon|max-lon=f' => \$max_lon,
  'min_height|e=i' => \$min_height,
  'min_lat|min-lat=f' => \$min_lat,
  'min_lon|min-lon=f' => \$min_lon,
  'multicast|m=s' => \$multicast,
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
  'vrep_start_port|s=i' => \$vrep_start_port,
  'unique' => \$unique,
  'udp|u=s' => \@udp,
  'update_vrep|vrep' => \$update_vrep,
  'verbose|v' => \$verbose
  ) or $help = "yes";

#check for help request  
if ($help)
{
  my $output = " 
$script purpose:

  Configures GAMS agent settings for a simulation

options:
  --agents|-a num        number of agents that need to be in simulation
  --buffer|-m meters     buffer in meters between agents
  --border r0 r1 ...     regions to put a border around
  --broadcast|b host     broadcast ip/host to use for network transport
  --distributed          for positions in region, distribute uniformly
  --duration|-t sec      max duration of simulation in secs
  --equidistant          alias to --distributed
  --first|-f num         first agent number (e.g 0, 1, 2, etc.)
  --gams_debug|-gd lev   log level for GAMS
  --help|-h              print guidance information
  --height_diff meters   height difference in meters when paired with unique
  --hz|-z hertz          periodic execution rate for agents
  --invert|-i            invert the x and y axis in a formation
  --last|-l num          last agent number (e.g. 1, 2, 3, etc.)
  --madara_debug|-md lev log level for MADARA
  --max_lat|max-lat x    defines the maximum latitude, generally for a region
  --max_lon|max-lon x    defines the maximum longitude, generally for a region
  --min_height|-e num    height in meters
  --min_lat|min-lat x    defines the minimum latitude, generally for a region
  --min_lon|min-lon x    defines the minimum longitude, generally for a region
  --multicast|m host     multicast ip/host to use for network transport
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
                         
  --vrep_start_port|-s # VREP port number to start from  
  --udp|u self h1 ...    udp ip/hosts to use for network transport 
  --unique|-u            requires unique attribute (e.g., height)
  --update_vrep|vrep     updates the VREP bounding box with a min|max-lat|lon
  --verbose|-v           print detailed debug info\n";
  
  print("$output\n");
}
else
{
  if ($verbose)
  {
    my $output = "
$script is using the following configuration:
  agents = " . ($agents ? $agents : 'no change') . "
  border = " . (scalar @border > 0 ?
    ("\n    " . join ("\n    ", @border)) : 'no') . "
  broadcast = " . ($broadcast ? $broadcast : 'no') . "
  buffer = $buffer meters
  equidistant = " . ($equidistant ? 'yes' : 'no') . "
  first = " . ($first ? $first : 'default') . "
  gams_debug = " . ($gams_debug ? $gams_debug : 'no change') . "
  height_diff = $height_diff
  hz = " . ($hz ? $hz : 'no change') . "
  invert = " . ($invert ? 'yes' : 'no') . "
  last = " . ($last ? $last : 'default') . "
  madara_debug = " . ($madara_debug ? $madara_debug : 'no change') . "
  max_lat = " . ($max_lat ? $max_lat : 'no change') . "
  max_lon = " . ($max_lon ? $max_lon : 'no change') . "
  min_height = " . ($min_height ? $min_height : 'no change') . "
  min_lat = " . ($min_lat ? $min_lat : 'no change') . "
  min_lon = " . ($min_lon ? $min_lon : 'no change') . "
  multicast = " . ($multicast ? $multicast : 'no') . "
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
  vrep_start_port = " . ($vrep_start_port ? $vrep_start_port : 'no change') . "
  udp = " . (scalar @udp > 0 ? ("\n    " . join ("\n    ", @udp)) : 'no') . "
  unique = " . ($unique ? 'yes' : 'no') . "\n";
  
    print("$output\n");
  }

  # recursively create path/name
  my $create_result = make_path("$path");
  
  if ($create_result)
  {
    $agents = $agents ? $agents : 1;
    $hz = $hz ? $hz : 1;
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
    open (env_file, '>', "$path/env.mf");
      print env_file $env_contents;
    close (env_file);
    
    if ($verbose)
    {
      print("Creating common initialization file...\n");
    }
    
    # create the common file
    open (common_file, '>', "$path/common.mf");
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
 * Set to .algorithm variable from common.mf
 * Feel free to change this to whatever you like
 **/
agent.$i.command = .algorithm;\n";
    
      open (agent_file, '>', "$path/agent_$i.mf");
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
\$duration = $duration;
\$madara_debug = $madara_debug;
\$gams_debug = $gams_debug;
\$period = $period;
\$agents = $agents;
\@border = (" . join(",", @border) . ");
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
      $run_contents .= "rename \"\$path/agent_$i.log\", ";
      $run_contents .= "\"\$path/agent_$i.prev.log\";\n";
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
    open (run_file, '>', "$path/run.pl");
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
      print ("Reading agents info from $path/run.pl\n");
    }
    
    my $run_contents;
    open run_file, "$path/run.pl" or
      die "ERROR: Couldn't open $path/run.pl\n"; 
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
        if (open run_file, "$path/agent_0.mf")
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
agent.$i.command = .algorithm;\n";

	        if (open (my $agent_file, '>', "$path/agent_$i.mf"))
          {
	          print $agent_file $agent_contents;
	          close ($agent_file);
          }
          else
          {
            print ("ERROR: Unable to write to file $path/agent_$i.mf");
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
              print ("  Deleting $path/agent_$i.mf...\n");
            }
    
            unlink "$path/agent_$i.mf";
          }
          if ($remove_logs)
          {
            if ($verbose)
            {
              print ("  Deleting $path/agent_$i.*.log...\n");
            }
    
            unlink "$path/agent_$i.log";
            unlink "$path/agent_$i.prev.log";
          }
        }
      }
      
      $run_contents =~ s/(agents\s*=\s*)\d+/$1$agents/;
      
      open run_file, ">$path/run.pl" or
        die "ERROR: Couldn't open $path/run.pl for writing\n";
        print run_file  $run_contents; 
      close run_file;
          
    }
    else
    {
      print ("ERROR: Couldn't find agents info in $path/run.pl\n");
    }
  }  
  
  if (!$first || !$last)
  {
    # if we have not defined agents, read the agent number or use default of 1
    if (!$agents)
    {
      my $run_contents;
      open run_file, "$path/run.pl" or
        die "ERROR: Couldn't open $path/run.pl\n"; 
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
    if (!$first)
    {
      $first = 0;
    }
    
    # default last is $agents - 1
    if (!$last)
    {
      $last = $agents - 1;
    }
    
    if ($verbose)
    {
      print ("Defaults init: first = $first, last = $last, agents=$agents\n");
    }
  }
  
  # check for default init for region
  if (($randomize or $equidistant) and !$region)
  {
    # read the contents of the $path/env.mf file
    my $env_contents;
    open env_file, "$path/env.mf" or
      die "ERROR: Couldn't open $path/env.mf\n"; 
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
  
    # read the contents of the $path/env.mf file
    my $env_contents;
    open env_file, "$path/env.mf" or
      die "ERROR: Couldn't open $path/env.mf\n"; 
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
    open (env_file, '>', "$path/env.mf");
      print env_file $env_contents;
    close (env_file);
  }
  
  if ($region)
  {
    # A region has been specified
    if ($verbose)
    {
      print ("Reading $path/env.mf for $region information\n");
    }
  
    # read the contents of the $path/env.mf file
    my $env_contents;
    open env_file, "$path/env.mf" or
      die "ERROR: Couldn't open $path/env.mf\n"; 
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
      open (env_file, '>', "$path/env.mf");
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
          open agent_file, "$path/agent_$i.mf" or
            die "ERROR: Couldn't open $path/agent_$i.mf for reading\n";
            $agent_contents = join("", <agent_file>); 
          close agent_file;
      
          if ($verbose)
          {
            print ("Replacing agent_$i.mf init pos with [$lat, $lon]\n");
          }
          
          # replace lat and lon with our new points
          $agent_contents =~ s/(\.initial_lat\s*=\s*)\-?\d+\.\d+/$1$lat/;
          $agent_contents =~ s/(\.initial_lon\s*=\s*)\-?\d+\.\d+/$1$lon/;
          
          open agent_file, ">$path/agent_$i.mf" or
            die "ERROR: Couldn't open $path/agent_$i.mf for writing\n";
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
          open agent_file, "$path/agent_$i.mf" or
            die "ERROR: Couldn't open $path/agent_$i.mf for reading\n";
            $agent_contents = join("", <agent_file>); 
          close agent_file;
      
          if ($verbose)
          {
            print ("  Replacing agent_$i.mf init pos with [$lat, $lon]\n");
          }
          
          # replace lat and lon with our new points
          $agent_contents =~ s/(\.initial_lat\s*=\s*)\-?\d+\.\d+/$1$lat/;
          $agent_contents =~ s/(\.initial_lon\s*=\s*)\-?\d+\.\d+/$1$lon/;
          
          open agent_file, ">$path/agent_$i.mf" or
            die "ERROR: Couldn't open $path/agent_$i.mf for writing\n";
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
      open agent_file, "$path/agent_$i.mf" or
        die "ERROR: Couldn't open $path/agent_$i.mf for reading\n";
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
      
      open agent_file, ">$path/agent_$i.mf" or
        die "ERROR: Couldn't open $path/agent_$i.mf for writing\n";
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
      open agent_file, "$path/agent_$i.mf" or
        die "ERROR: Couldn't open $path/agent_$i.mf for reading\n";
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
      
      open agent_file, ">$path/agent_$i.mf" or
        die "ERROR: Couldn't open $path/agent_$i.mf for writing\n";
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
    open run_file, "$path/run.pl" or
      die "ERROR: Couldn't open $path/run.pl\n"; 
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
      print ("  Writing back to $path/run.pl\n");
    }
    
    # write the updated info to the run file
    open run_file, ">$path/run.pl" or
      die "ERROR: Couldn't open $path/run.pl for writing\n";
      print run_file  $run_contents; 
    close run_file;
  }
  
  # check if the user wants to change transports
  if ($duration)
  {
    if ($verbose)
    {
      print ("Changing duration of sim in $path/run.pl...\n");
    }
    
    my $run_contents;
    open run_file, "$path/run.pl" or
      die "ERROR: Couldn't open $path/run.pl\n"; 
      $run_contents = join("", <run_file>); 
    close run_file;
    
    $run_contents =~ s/(duration\s*=\s*)\d+/$1$duration/;  
    
    open run_file, ">$path/run.pl" or
      die "ERROR: Couldn't open $path/run.pl for writing\n";
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
    
    open run_file, "$path/run.pl" or
      die "ERROR: Couldn't open $path/run.pl\n"; 
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
      print ("  Writing back to $path/run.pl\n");
    }
    
    open run_file, ">$path/run.pl" or
      die "ERROR: Couldn't open $path/run.pl for writing\n";
      print run_file  $run_contents; 
    close run_file;
  }
  
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
