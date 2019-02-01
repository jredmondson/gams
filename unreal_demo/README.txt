
INTRO:

  This directory has been created by the gpc.pl script and contains
  a custom simulation.

HOW TO:

  COMPILE ON LINUX:
  
    mwc.pl -type gnuace workspace.mwc
    make vrep=1
    
  COMPILE ON WINDOWS:
  
    mwc.pl -type vc12 workspace.mwc 
    
    <Open Visual Studio and compile project>. Note that you should compile the
    solution in the same settings you used for ACE, MADARA, and GAMS. For most
    users, this is probably Release mode for x64. However, whatever you use
    make sure that you have compiled ACE, MADARA, and GAMS in those settings.
    
  RUN THE SIMULATION:
    open VREP simulator
    perl sim/run.pl 

  RUN UNREAL SIMULATION:
    open Unreal with an AirSim plugin
    perl sim/run_unreal_sim.pl
    
