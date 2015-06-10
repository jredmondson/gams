#!/bin/bash
# rebuild GAMS...useful when swapping branches

cd $GAMS_ROOT
make realclean -j $CORES
perl $ACE_ROOT/bin/mwc.pl -type gnuace gams.mwc
make realclean -j $CORES
make vrep=1 -j $CORES
