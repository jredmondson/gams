#!/usr/bin/env python
import sys

def print_json(x):
    print """ {
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": { """
    for i in range(0, int(x)):
        # Handle all cases up to and before i == x - 1 (e.g everything before the last) 
        if (i < (int(x) - 1)):
            template = """ "agent.%d": {
              "VehicleType": "SimpleFlight",
              "X": %d,
              "Y": 0,
              "Z": -2,
              "Yaw": -180
            }, """ % (i, i)
        # Handle the ending comma. Can be done easier, this is just simpler code to read.
        # In the case of agent == 1 passed in, it will default to this, and so no comma.
        elif (i == (int(x) - 1)):
            template = """ "agent.%d": {
              "VehicleType": "SimpleFlight",
              "X": %d,
              "Y": 0,
              "Z": -2,
              "Yaw": -180
            } """ % (i, i)
        print template
    print """ } \n } """
    return

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print "Usage: python settings_json.py madara|json num_agents"
    elif sys.argv[1] == "json":
        print_json(sys.argv[2])
