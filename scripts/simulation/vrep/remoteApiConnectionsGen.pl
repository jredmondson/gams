#!/usr/bin/perl
use strict;
use Scalar::Util qw(looks_like_number);

#Script to generate the remote API Ports in the range [start_port, start_port+num_ports] 
#Usage: perl remoteApiConnectionsGen.pl START_PORT NUM_PORTS

# get arguments
my $num_args = $#ARGV + 1;
if ($num_args != 2)
{
  print("usage: remoteApiConnectionsGen.pl <start_port> <num_ports>\n");
  exit;
}
my ($start_port, $num_ports) = @ARGV;
if (!looks_like_number($start_port))
{
  print("first argument must be a number\n");
  print("usage: remoteApiConnectionsGen.pl <start_port> <num_ports>\n");
  exit;
}
if (!looks_like_number($num_ports))
{
  print("second argument must be a number\n");
  print("usage: remoteApiConnectionsGen.pl <start_port> <num_ports>\n");
  exit;
}
  
print("creating $num_ports starting at port $start_port\n");
my $osname = $^O;
my $vrep_root = $ENV{"VREP_ROOT"};

#File output path is dependent on operating system
my $target_path;
if ($osname eq "MSWin32") # windows default
{
  print("detected Windows\n");
  $target_path = $vrep_root; 
}
elsif ($osname eq "linux") # linux default
{
  print("detected Linux\n");
  $target_path = $vrep_root;
}
elsif ($osname eq "darwin") # Mac OS X default
{
  print("detected Mac OS\n");
  $target_path = "$vrep_root/vrep.app/Contents/MacOS";
}
else
{
  print("OS not recognized...printing to pwd\n");
  $target_path = cwd();
}

my $target_file = "$target_path/remoteApiConnections.txt";
print("creating file at $target_file\n");

open(file, ">$target_file"); 

print(file "// This file defines all the continuous remote API server\n"); 
print(file "// services (started at remote API plugin initialization, i.e. V-REP start-up)\n");
print(file "// This file was generated using $0\n\n");

for(my $i = 0; $i < $num_ports; $i++)
{
  print(file "portIndex", $i+1, "_port            = ", $start_port+$i, "\n");     
  print(file "portIndex", $i+1, "_debug           = false\n");
  print(file "portIndex", $i+1, "_syncSimTrigger  = true\n\n");
}
