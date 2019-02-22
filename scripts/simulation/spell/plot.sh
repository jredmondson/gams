#!/bin/bash


#getting number of members
membersKey='group.nodes.members.size='
membersValue=`grep $membersKey madara_init_common.mf`
membersValue=${membersValue#"$membersKey"}
membersValue=${membersValue%;}


membersInGroup=3

`cp config_template.yaml config.yaml`
plotsString=""
iterations=$(($membersValue/$membersInGroup))
pointsPerPlot=40
for ((plotNumber=0; plotNumber<$iterations ;++plotNumber)); do
  first=$((plotNumber*$membersInGroup))
  second=$(($first+$membersInGroup-1))
  echo "  agent."$first"-"$second".location:" >> config.yaml
  echo "    plot_1:" >> config.yaml
  echo "       0: 'x'" >> config.yaml
  echo "       1: 'y'" >> config.yaml
  echo "    points_per_plot: "$pointsPerPlot"" >> config.yaml
  echo "" >> config.yaml
done


python $GAMS_ROOT/port/python/tools/plotting_sample.py config.yaml
