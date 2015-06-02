#!/usr/bin/perl
use strict;

#Script to generate the remote API Ports in the range [start_port, start_port+num_ports] 
#Usage: perl remoteApiConnectionsGen.pl START_PORT NUM_PORTS
  
  # get arguments
  my ($start_port, $num_ports) = @ARGV;
  my $osname = $^O;
  my $vrep_root = $ENV{"VREP_ROOT"};
  
  #File output path is dependent on operating system
  my $target_path;
    if ($osname eq "MSWin32") # windows default
    {
	$target_path = $vrep_root; 
    }
    elsif ($osname eq "linux") # linux default
    {
	$target_path = $vrep_root;
    }
    elsif ($osname eq "darwin") # Mac OS X default
    {
	$target_path = $vrep_root;
    }
    else
    {
      print("OS not recognized... exiting");
      exit(1);
    }

open(file,">$target_path/remoteApiConnections.txt"); 

print(file "This file defines all the continuous remote API server"); 
print(file "services (started at remote API plugin initialization, i.e. V-REP start-up)\n");
print(file "This file was generated using $0\n\n");

for(my $i = 0; $i < $num_ports; $i++)
{

	print(file "portIndex", $i+1, "_port	      \t\t= ", $start_port+$i, "\n");   	
	print(file "portIndex", $i+1, "_debug	      \t= false\n");
	print(file "portIndex", $i+1, "_syncSimTrigger	= true\n\n");
}

