#!/bin/python
import sys

def print_madara(x):
    for i in range(0, int(x)):
        template = """ group.nodes.members.%d="agent.%d"; """ % (i, i)
        print template
    return

def print_json(x):
    print """ {
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": { """
    for i in range(0, int(x)):
        template = """ "agent.%d": {
          "VehicleType": "SimpleFlight",
          "X": %d,
          "Y": 0,
          "Z": -2,
          "Yaw": -180
        }, """ % (i, i)
        print template
    print """ } \n } """
    return

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print "Usage: python settings_json.py madara|json num_agents"
    elif sys.argv[1] == "madara":
        print_madara(sys.argv[2])
    elif sys.argv[1] == "json":
        print_json(sys.argv[2])
