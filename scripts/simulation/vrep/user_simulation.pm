use strict;
use Data::Dumper;
use File::Basename;
package user_simulation;

# empty string to represent not set
my $term_prefix = $ENV{"GAMS_TERM_PREFIX"};
my $term_suffix = $ENV{"GAMS_TERM_SUFFIX"};

sub run {
  # get arguments
  my %args = @_;

  my ($agents, $duration, $period, $dir, $madara_debug, $gams_debug,
      @border, $coverages, $launch, $num_udps, $controller, $domain, $mc);
    
  # get environment variables
  my $osname = $^O;
  my $vreproot = $ENV{"VREP_ROOT"};
  my $gamsroot = $ENV{"GAMS_ROOT"};
  
  $agents = $args{agents};
  
  $duration = $args{duration} ? $args{duration} : 300;
  
  $domain = $args{domain} ? $args{domain} : "gams_sims";
  
  $period = $args{period} ? $args{period} : 1;
  
  $dir = $args{dir} ? $args{dir} : '.';
  
  $controller = $args{controller} ?
    $args{controller} : "$gamsroot/bin/gams_controller";
  
  $madara_debug = $args{madara_debug} ? $args{madara_debug} : 1;
  $gams_debug = $args{gams_debug} ? $args{gams_debug} : 1;
  $mc = $args{mc} ? $args{mc} : 1;
  

  if ($args{coverages})
  {
    $coverages = $args{coverages};
  }
  
  if ($args{launch})
  {
    $launch = $args{launch};
  }
  
  if ($args{udp})
  {
    $num_udps = scalar @{$args{udp}};
  }
  
  $launch = 1;
    
  if ($launch == 1)
  {
    for (my $i=0; $i < $agents; $mc > 1 ? $i += $mc : $i++)
    {
	    my $common_args = "";
	  
	    #args common to all operating systems
      $common_args = "$controller ";
	    $common_args .= "-i $i -n $agents ";
      
      if ($args{multicast})
      {
        $common_args .= "-m " . @{$args{multicast}}[0] . " ";
      }
      elsif ($args{broadcast})
      {
        $common_args .= "-b " . @{$args{broadcast}}[0] . " ";
      }
      elsif ($args{udp})
      {
        if ($agents > $num_udps - 1)
        {
          die "ERROR: num agents is $agents. " .
            " UDP only has " . ($num_udps) . " addresses. " .
            " You must have at least " . ($agents + 1) . " ips/hosts.\n";
        }
        
        print "\$args{udp} = " . join (", ", @{$args{udp}}) . "\n";
          
        print "scalar \$args{udp} = $num_udps\n";
           
        # we could try shifting, splicing, etc., but that's going to be
        # very slow. Instead, we'll just treat the udp_list as a circular
        # buffer. This should be the fastest way to offset into the list
        # based on the self ip of the agent.
        for (my $j = 0; $j < $num_udps; ++$j)
        {
          # we could use modulus, but that's slower than just subtracting
          # agents to accomplish the same thing (finding the remainder)
          
          my $offset = $j + $i;
          
          if ($offset >= $num_udps)
          {
            $offset -= $num_udps;
          }
          
          $common_args .= "-u " . @{$args{udp}}[$offset] . " ";
        }
      }
      elsif ($args{nt})
      {
        # user has selected no transport
        $common_args .= "-nt ";
      }
      
	    $common_args .= "--merge-controllers $mc ";
	    $common_args .= "--domain $domain ";
	    $common_args .= "--loop-time $duration --period $period ";
	    $common_args .= "--queue-length 2000000 ";
      $common_args .= "--madara-level $madara_debug ";
	    $common_args .= "--gams-level $gams_debug ";
      $common_args .= "--logfile agent_$i.log ";
	    $common_args .= " --madara-file ";
      $common_args .= "$dir/env.mf ";
      $common_args .= "$dir/common.mf ";

      if ($mc <= 1)
      {
        $common_args .= "$dir/agent_$i.mf ";
      }
	  
	    #command for POSIX operating systems
      my $cmd = "\"";	  
      $cmd .= $common_args;
	    $cmd .= "\"";
	  
	    #if term prefix was specified, concatenate prefix and suffix
      if ($term_prefix)
      {
	      print("$term_prefix $cmd $term_suffix");
        system("$term_prefix $cmd $term_suffix");
      }
	    #if windows
      elsif ($osname eq "MSWin32")
      {
	      # windows does not require quotes and uses different slashes
        $cmd = $common_args;
		    $cmd =~ s{/}{\\}g;
		
        print("start \"Agent$i\" /REALTIME $cmd\n\n");
        system("start \"Agent$i\" /REALTIME $cmd");
      }
      elsif ($osname eq "linux") # linux default
      {
	      print("xterm -hold -e $cmd &");
        system("xterm -hold -e $cmd &");
      }
      elsif ($osname eq "darwin") # Mac OS X default
      {
        print(
		      "osascript $gamsroot/scripts/common/mac_launch_terminal.scpt $cmd");
        system(
		      "osascript $gamsroot/scripts/common/mac_launch_terminal.scpt $cmd");
      }
      else
      {
        print("OS not recognized and \$term_prefix is not set...exiting");
        exit(1);
      }
    }
  }
 
  # create command for dynamic simulation launch
  my $cmd = "$gamsroot/bin/dynamic_simulation -t -1 --sim-time-poll-rate 2 ";
  $cmd .= "-n $agents ";
	$cmd .= "--domain $domain ";
  $cmd .= "--madara-file $dir/env.mf ";
  
  if ($args{multicast})
  {
    $cmd .= "-m " . @{$args{multicast}}[0] . " ";
  }
  elsif ($args{broadcast})
  {
    $cmd .= "-b " . @{$args{broadcast}}[0] . " ";
  }
  elsif ($args{udp} and $num_udps > 0)
  {
    # we could try shifting, splicing, etc., but that's going to be
    # very slow. Instead, we'll just treat the udp_list as a circular
    # buffer. This should be the fastest way to offset into the list
    # based on the self ip of the agent.
    for (my $j = 0; $j < $num_udps; ++$j)
    {
      # we could use modulus, but that's slower than just subtracting
      # agents to accomplish the same thing (finding the remainder)
        
      my $offset = $j + $agents;
          
      if ($offset >= $num_udps)
      {
        #offset wraps around the array
        $offset -= $num_udps;
      }
        
      $cmd .= "-u " . @{$args{udp}}[$offset] . " ";
    } #end for loop through UDP args
  } # end if udp
      
  # do we need to draw a border around a region?
  if ($args{border} and scalar @{$args{border}} > 0)
  {
    $cmd .= " --border " . join (', ', @{$args{border}}) . " ";
  }
  
  if ($coverages)
  {
    $cmd .= " -c $coverages ";
  }
  
  if (!$args{unreal})
  {
    # launch dynamic_simulation
    if ($osname eq "MSWin32")
    {
      $cmd =~ s{/}{\\}g;
      print("start \"DeviceSimulator\" /REALTIME $cmd");
      system("start \"DeviceSimulator\" /REALTIME $cmd");
    }
    else
    {
      print("$cmd");
      system("$cmd");
    }
  }
}

1;
