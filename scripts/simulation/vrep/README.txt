If at any point your simulation begins to fail and loads "debug" or
some other algorithm, simply run the following from a terminal or
command line.

karl -m "239.255.0.1:4150" -i madara_init_common.mf -k

Note that you can also change letters by doing something similar. For example:

karl -m "239.255.0.1:4150" -k "swarm.algorithm.args.text = 'NAT'; group.nodes.members.size=9; group.nodes.members.0='agent.0'; group.nodes.members.1='agent.1'; group.nodes.members.2='agent.2'; group.nodes.members.3='agent.3'; group.nodes.members.4='agent.4'; group.nodes.members.5='agent.5'; group.nodes.members.6='agent.6'; group.nodes.members.7='agent.7'; group.nodes.members.8='agent.8'; group.nodes.members.9='agent.9'; group.nodes.members.10='agent.10'; group.nodes.members.11='agent.11'; swarm.algorithm = 'spell'; swarm.algorithm.args.group = 'group.nodes'; swarm.algorithm.args.origin = region.0.0;"
