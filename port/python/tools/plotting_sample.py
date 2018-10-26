import config_parser
import sys



# takes 2 args
if len(sys.argv) >= 2:

    plotters = config_parser.create_plotters_from_config(sys.argv[1])

    while True:
        for key in plotters:
            plotters[key].visualize()

else:
    print "Error! no config file specified"
    print "Please pass the config file path as the argument when running"

