OVERVIEW

  In this directory are scripts for generating and configuring GAMS projects.

GAMS PROJECT CONFIGURATOR (GPC) SCRIPT
  
  The gpc script is a dynamic tool for creating and configuring portable 
  GAMS projects on Windows, Linux, Mac, Android, and other architectures.
  Among the core features of the script are the generation and maintenance
  of the following GAMS/MADARA features:
  
  CORE FEATURE LIST
  
  1) Threads with the -nt or --new-thread argument. This creates a new MADARA
     thread in a custom controller for GAMS applications.
  2) Algorithms with the -na or --new-algorithm argument. This creates a new 
     GAMS algorithm in a custom controller for GAMS applications.
  3) Platforms with the -np or --new-platform argument. This creates a new
     GAMS platform in a custom controller for GAMS applications.
  4) Transports with the -nr or --new-transport argument. This creates a new
     networking transport in a custom controller for GAMS applications.
  5) Simulation configuration with a variety of options, including specifying
     algorithms (-a or --algorithm), the number of agents (--agents)  
     
  For more comprehensive help, please access the gpc's help system with -h or
  --help.
     