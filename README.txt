0. ABOUT GAMS

The Group Autonomy for Mobile Systems (GAMS) project at Carnegie Mellon University
is intended to provide a distributed operating environment for control of unmanned
autonomous systems (UAS). The repository is composed of C++, Java, MADARA, and some
LUA bindings that enable a single person to control and understand information from
a swarm of devices, robots, or UAS.

GAMS is an extension of an earlier project called SMASH.


1. How it works

GAMS is built on top of MADARA, a distributed reasoning engine that was created for
real-time systems that require quality-of-service, fast execution times, and minimal
memory footprint--despite intermittent connections. MADARA provides reasoning services
for the UAS to process control messages from a nearby user or contextual updates from
other members of the swarm. MADARA also provides filters for advanced synthesis and
fusion of information.


2. How you can interact with the system

The easiest way to interact with the system is by installing MADARA. Through MADARA,
you can update or read global variables that the UAVs are keyed on. This is the most
basic interaction mechanism possible with the GAMS system.

More advanced users can download this code repository and create simulations and
extensions of the GAMS code base to perform new distributed AI.


3. How you can get involved

If you are interested in working on a real-time, context-based distributed operating
environment for UAS, feel free to contact Dr. James Edmondson. 
