use strict;
package user_simulation;

# empty string to represent not set
my $term_prefix = $ENV{"GAMS_TERM_PREFIX"};
my $term_suffix = $ENV{"GAMS_TERM_SUFFIX"};

sub run {

  # get arguments
  my ($num, $time, $period, $dir, $madara_debug, $gams_debug,
      $border, $num_coverages, $launch_controllers) = @_;
	  
  # get environment variables
  my $osname = $^O;
  my $vreproot = $ENV{"VREP_ROOT"};
  my $gamsroot = $ENV{"GAMS_ROOT"};

  if ($launch_controllers == 1)
  {
    for (my $i=0; $i < $num; $i++)
    {
	  my $common_args = "";
	  
	  #args common to all operating systems
      $common_args = "$gamsroot/bin/gams_controller ";
	  $common_args .= "-i $i -n $num ";
	  $common_args .= "--loop-time $time --period $period ";
	  $common_args .= "--queue-length 2000000 ";
      $common_args .= "--madara-level $madara_debug ";
	  $common_args .= "--gams-level $gams_debug ";
      $common_args .= "--logfile agent_$i.log ";
	  $common_args .= " --madara-file ";
      $common_args .= "$dir/env.mf ";
      $common_args .= "$dir/common.mf ";
      $common_args .= "$dir/agent_$i.mf ";
	  
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
		
        print("start \"Device$i\" /REALTIME $cmd\n\n");
        system("start \"Device$i\" /REALTIME $cmd");
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
  $cmd .= "-n $num ";
  $cmd .= "--madara-file $dir/env.mf";
  
  # do we need to draw a border around a region?
  if ($border)
  {
    $cmd .= " -b $border";
  }
  
  if ($num_coverages)
  {
    $cmd .= " -c $num_coverages";
  }
  
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

1;
