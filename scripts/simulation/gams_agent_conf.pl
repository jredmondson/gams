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
my $equidistant;
my $buffer = 5;
my $first = 0;
my $gams_debug;
my $help;
my $hz;
my $invert;
my $last = 1;
my $madara_debug;
my $height_diff = 1;
my $ordered;
my $path = '.';
my $permute;
my $platform;
my $randomize;
my $region;
my $rotate;
my $starting_port;
my $min_height;
my $unique;
my $verbose;

# setup options parser
GetOptions(
  'buffer|b=i' => \$buffer,
  'equidistant|distributed' => \$equidistant,
  'first|f=i' => \$first,
  'gams_debug|g=i' => \$gams_debug,
  'help|h' => \$help,
  'height_diff=i' => \$height_diff,
  'hz|z=i' => \$hz,
  'invert|i' => \$invert,
  'last|l=i' => \$last,
  'madara_debug|m=i' => \$madara_debug,
  'min_height|e=i' => \$min_height,
  'ordered|o' => \$ordered,
  'path|p=s' => \$path,
  'permute|u=s' => \$permute,
  'platform|l=s' => \$platform,
  'randomize|a' => \$randomize,
  'region|r=s' => \$region,
  'rotate|t' => \$rotate,
  'starting_port|s=i' => \$starting_port,
  'unique|u' => \$unique,
  'verbose|v' => \$verbose
  ) or $help = "yes";

#check for help request  
if ($help)
{
  my $output = " 
$script purpose:

  Configures GAMS agent settings for a simulation

options:
  --buffer|-m meters    buffer in meters between agents
  --distributed         for positions in region, distribute uniformly
  --equidistant         alias to --distributed
  --first|-f num        first agent number (e.g 0, 1, 2, etc.)
  --gams_debug|-g level log level for GAMS
  --help|-h             print guidance information
  --height_diff meters  height difference in meters when paired with unique
  --hz|-z hertz         periodic execution rate for agents
  --invert|-i           invert the x and y axis in a formation
  --last|-l num         last agent number (e.g. 1, 2, 3, etc.)
  --madara_debug|-m lev log level for MADARA
  --min_height|-e num   height in meters
  --ordered             order distribution by agent id top->bottom, left->right
  --path|-p directory   the directory path to agent settings
  --permute|-u          Permute existing locations or heights 
  --platform|-l model   GAMS platform model. Options are:
  
                        vrep-quad | vrep-uav : quadcopter
                        vrep-boat            : boat
                        vrep-ant             : ant robot
                        vrep-summit          : Summit robot
            
  --randomize|-a        Randomize the target locations or heights 
  --rotate|-t           Rotate a formation 
  --region|-r region    Region name for starting locations     
  --starting_port|-s #  VREP port number to start from   
  --unique|-u           Requires unique attribute (e.g., height)
  --verbose|-v          print detailed debug info\n";
  
  print("$output\n");
}
else
{
  if ($verbose)
  {
    my $output = "
$script is using the following configuration:
  buffer = $buffer meters
  equidistant = " . ($equidistant ? 'yes' : 'no') . "
  first = $first
  gams_debug = " . ($gams_debug ? $gams_debug : 'no change') . "
  height_diff = $height_diff
  hz = " . ($hz ? $hz : 'no change') . "
  invert = " . ($invert ? 'yes' : 'no') . "
  last = $last
  madara_debug = " . ($madara_debug ? $madara_debug : 'no change') . "
  min_height = " . ($min_height ? $min_height : 'no change') . "
  ordered = " . ($ordered ? 'yes' : 'no') . "
  path = $path
  platform = " . ($platform ? $platform : 'no change') . "
  randomize = " . ($randomize ? 'yes' : 'no') . "
  rotate = " . ($rotate ? 'yes' : 'no') . "
  region = " . ($region ? $region : 'none specified') . "
  starting_port = " . ($starting_port ? $starting_port : 'no change') . "
  unique = " . ($unique ? 'yes' : 'no') . "\n";
  
    print("$output\n");
  }
  
  if ($region)
  {
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
    my $min_lat;
    my $min_lon;
    my $max_lat;
    my $max_lon;
    
    # find region size
    if ($env_contents =~ m{$region\.size\s*=\s*(\d+)})
    {
      my $region_size = $1;
      my $num_agents = $last - $first + 1;
    
      if ($verbose)
      {
        print ("Found $region.size ($region_size)\n");
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
            print ("Found $region.$i with [$point[0], $point[1]]\n");
          }
          
          $vertices[$i][0] = $point[0];
          $vertices[$i][1] = $point[1];
        } # end found region
        else
        {
          die "ERROR: Region $region is malformed. Bad vertices\n";
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
            print ("Formation will be inverted and rotated...\n")
          }
          elsif ($invert)
          {
            print ("Formation will be inverted...\n")
          }
          elsif ($rotate)
          {
            print ("Formation will be rotated...\n")
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
            print ("Generated [$lat, $lon] for agent $i...\n");
          }
        
          # until we have a valid point, keep trying
          while (!inside_polygon (\@point, \@vertices))
          {
            if ($verbose)
            {
              print ("[$lat, $lon] not in region $region. Regenerating...\n");
            }
        
            $lat = ($min_lat + rand ($max_lat - $min_lat));
            $lon = ($min_lon + rand ($max_lon - $min_lon));
            @point = ($lat, $lon);
          
            if ($verbose)
            {
              print ("Generated [$lat, $lon] for agent $i...\n");
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
            print ("Replacing agent_$i.mf init pos with [$lat, $lon]\n");
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
  
  if ($starting_port)
  {
    if ($verbose)
    {
      print ("Starting port specified. Redoing VREP ports.\n");
    }
  
    # Change port numbers in the agent range
    for (my $i = $first, my $j = $starting_port; $i <= $last; ++$i, ++$j)
    {
      # we have a valid point. Change the agent's init file.
      my $agent_contents;
      open agent_file, "$path/agent_$i.mf" or
        die "ERROR: Couldn't open $path/agent_$i.mf for reading\n";
        $agent_contents = join("", <agent_file>); 
      close agent_file;
  
      if ($verbose)
      {
        print ("Replacing agent_$i.mf vrep_port with $j\n");
      }
      
      # replace port
      $agent_contents =~ s/(\.vrep_port\s*=\s*)\d+/$1$j/;
      
      if ($verbose)
      {
        print ("Writing change back to agent_$i.mf\n");
      }
      
      open agent_file, ">$path/agent_$i.mf" or
        die "ERROR: Couldn't open $path/agent_$i.mf for writing\n";
        print agent_file  $agent_contents; 
      close agent_file;
    } #end agent $first -> $last
  } #end $starting port
  
  
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
        print ("Replacing agent_$i.mf initial_alt with $j\n");
      }
      
      # replace port
      $agent_contents =~ s/(\.initial_alt\s*=\s*)\d+/$1$j/;
      
      if ($verbose)
      {
        print ("Writing change back to agent_$i.mf\n");
      }
      
      open agent_file, ">$path/agent_$i.mf" or
        die "ERROR: Couldn't open $path/agent_$i.mf for writing\n";
        print agent_file  $agent_contents; 
      close agent_file;
      
      if ($unique)
      {
        if ($verbose)
        {
          print ("Unique modifier specified. Increasing by $height_diff.\n");
        }
      
        $j += $height_diff;
      }
    } #end agent $first -> $last
  } #end $starting port
  
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
  
  if ($verbose)
  {
    print "inside_polygon: x = $x, y = $y\n";
  }
  
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
    print ("inside_polygon: [$x, $y] IS inside polygon.\n");
  }
  
  return $result;
}
