#!/bin/sh

for i in `seq 1 8`;
do
  rosrun ros_messaging loop_rate_testing -t 1 -n "test$i" 2> "$i.txt" &
done
