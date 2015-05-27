use strict;
package profiling;

# empty string to represent not set
my $term_prefix = "";
my $term_suffix = "";

sub run {
  # get arguments
  my ($scenario, $num, $time, $period) = @_;
  my $osname = $^O;
  
  # launch drone controllers
  my $gams_root = $ENV{"GAMS_ROOT"};
  print("$gams_root\n");
  for (my $i=0; $i < $num; $i++)
  {
    my $cmd = "\"$gams_root/gams_controller -m 239.255.0.1:4150 -i $i -p null --loop-time $time --period $period --madara-file $gams_root/scripts/profiling/$scenario/madara_init_$i.mf\"";
    if ($term_prefix)
    {
      system("$term_prefix $cmd $term_suffix");
    }
    elsif ($osname eq "MSWin32") # windows default
    {
#      $cmd = "$gams_root\\bin\\gams_controller -i $i -n $num -p vrep --loop-time $time --period $period --madara-file $gams_root\\scripts\\simulation\\$sim\\madara_init_$i.mf $gams_root\\scripts\\simulation\\areas\\$area.mf $gams_root\\scripts\\simulation\\madara_init_common.mf -l $debug";
#      print("start \"Device$i\" /REALTIME $cmd\n\n");
#      system("start \"Device$i\" /REALTIME $cmd");
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
}

1;
