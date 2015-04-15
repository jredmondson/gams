#!/bin/bash

duration=12
ip=192.168.0.142
port=40000
sleep_time=100
#cmd="./test_madara_writer -d $duration -s $sleep_time "
cmd="./test_madara_writer -d $duration -s $sleep_time -m 239.255.0.1:4150"
#for i in `seq 1 32`;
#do
#  cmd="$cmd -u $ip:$port"
#  ((port++))
#done

echo $cmd
$cmd
