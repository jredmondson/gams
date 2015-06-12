use strict;
package simulation;

# empty string to represent not set
my $term_prefix = "";
my $term_suffix = "";

sub run {
  # get arguments
  my ($num, $time, $period, $sim, $area, $madara_debug, $gams_debug, $border, $num_coverages) = @_;
  my $osname = $^O;
  my $vreproot = $ENV{"VREP_ROOT"};

  print("$^O\n");
  
  #launch the VREP simulator
  #if ($osname eq "MSWin32")
  #{
  #  my $cmd = "$vreproot\\vrep.exe";
  #  system("taskkill /im vrep.exe");
  #  system("start \"vrep\" /D $vreproot /REALTIME $cmd");
  #  system("timeout 10");
  #}
  #else
  #{
  #  system("vrep &");
  #  sleep(10);
  #}
  
  # launch drone controllers
  my $gams_root = $ENV{"GAMS_ROOT"};
  #$gams_root =~ s/\\/\//g;
  print("$gams_root\n");
  for (my $i=0; $i < $num; $i++)
  {
    my $cmd = "\"$gams_root/gams_controller -i $i -n $num -p vrep_boat --loop-time $time --period $period --madara-file $gams_root/scripts/simulation/$sim/madara_init_$i.mf $gams_root/scripts/simulation/areas/$area.mf $gams_root/scripts/simulation/madara_init_common.mf --madara-level $madara_debug --gams-level $gams_debug\"";
    if ($term_prefix)
    {
      system("$term_prefix $cmd $term_suffix");
    }
    elsif ($osname eq "MSWin32") # windows default
    {
      $cmd = "$gams_root\\bin\\gams_controller -i $i -n $num -p vrep --loop-time $time --period $period --madara-file $gams_root\\scripts\\simulation\\$sim\\madara_init_$i.mf $gams_root\\scripts\\simulation\\areas\\$area.mf $gams_root\\scripts\\simulation\\madara_init_common.mf --madara-level $madara_debug --gams-level $gams_debug";
      print("start \"Device$i\" /REALTIME $cmd\n\n");
      system("start \"Device$i\" /REALTIME $cmd");
    }
    elsif ($osname eq "linux") # linux default
    {
      system("xterm -hold -e $cmd &");
    }
    elsif ($osname eq "darwin") # Mac OS X default
    {
      system("osascript $gams_root/scripts/common/mac_launch_terminal.scpt $cmd");
    }
    else
    {
      print("OS not recognized and \$term_prefix is not set...exiting");
      exit(1);
    }
  }
 
  # launch simulation controller
  my $cmd = "$gams_root/bin/dynamic_simulation -n $num -s water --madara-file $gams_root/scripts/simulation/areas/$area.mf";
  if ($border)
  {
    $cmd = "$cmd -b $border";
  }
  if ($num_coverages)
  {
    $cmd = "$cmd -c $num_coverages";
  }
  
  if ($osname eq "MSWin32")
  {
    system("start \"DeviceSimulator\" /REALTIME $cmd");
  }
  else
  {
    system("$cmd");
  }
}

1;
