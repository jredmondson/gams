These scripts can be used to command agents in UnrealGAMS. We attempt to use
intelligent defaults that scale to N agents, but this may not be appropriate
for your demo needs. To modify, consider the following tweakable parameters.

*****************************************
20m_square.mf | run_20m_square.sh
*****************************************

```bash
./run 20m_square.sh [num agents] [no-transport]

or

gams_controller -p osc -M /home/jredmondson/projects/gams/scripts/simulation/unreal/move/20m_square.mf
```

This scripts uses an .origin variable combined with the .id agent identifier
to select a central point for the agent to loiter around with 4 waypoints in
a 20m square, centered at the .origin. To change the behavior, consider
changing the .origin to be something else.


*****************************************
stop.mf | run_stop.sh
*****************************************

```bash
./run_stop.sh [num agents] [no-transport]

or

gams_controller -p osc -M /home/jredmondson/projects/gams/scripts/simulation/unreal/move/stop.mf
```

Stops the agent's movements. Useful if you need to cancel another movement
script.


*****************************************
origin.mf | run_return.sh
*****************************************

```bash
./run_return.sh [num agents] [no-transport]

or

gams_controller -p osc -M /home/jredmondson/projects/gams/scripts/simulation/unreal/move/origin.mf
```

Move to the origin location.

This scripts uses an .origin variable combined with the .id agent identifier
to select a central point for the agent to return to. To change the behavior,
consider changing the .origin to be something else.
