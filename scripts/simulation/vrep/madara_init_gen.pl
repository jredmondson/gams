#!/usr/bin/perl
use strict;

#Script to generate madara init files
#Expects a template file using the following keywords
#   PORT  Designates the position to insert the port number
#   ID    Designates the position to insert the device id
#   POSX  Designates the position to insert the x coordinate
#   POSY  Designates the position to insert the y coordinate
# The function used to specify the x and y coordinates should be
# placed within this file in the subroutine get_coordinates below
 
#Place your coordinate functions here
sub get_coordinates
{
  my $ind = shift;
  my $x = $ind*5;
  my $y = $ind*2;
  return ($x, $y);

} 
my ($start_port, $num_files, $template_file, $target_path) = @ARGV;
print("creating $num_files init files in $target_path\n");
print("using $template_file as template\n");

for(my $i = 0; $i < $num_files; $i++)
{
  my $filename = "$target_path/madara_init_$i.mf";
  open(opfile, ">$filename") or die "Unable to open ouput file $filename";
  open(ipfile, "<$template_file") or die "Unable to open input file $template_file"; 
  while( my $line = <ipfile> )
  {
    my ($x, $y) = get_coordinates($i);
    my $port = $start_port+$i;
    $line =~ s/PORT/$port/;
    $line =~ s/POSX/$x/;
    $line =~ s/POSY/$y/;
    $line =~ s/IND/$i/;
    print(opfile $line);
  }
  close(ipfile);
  close(opfile);
}

